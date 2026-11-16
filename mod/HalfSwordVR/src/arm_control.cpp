// Motion controls: the controllers move the character's arms. See arm_control.hpp for the idea.
//
// Order of events in one frame (game thread):
//  1. arm_game_tick (before the world ticks): read both controllers, put them into the game
//     world the way the eyes are put there, and feed the game's aim from the sword hand.
//  2. The driver skeleton animates as usual (ABP_Willie).
//  3. Just before each of the character's other skeletal meshes runs its update (where
//     CharacterMesh0's CopyPoseFromMesh reads the driver's bone transforms), our hook on that
//     mesh's tick function bends the driver's arm bones: two-bone IK from the shoulder to the
//     controller, elbow on its own hinge, hand turned like the controller.
//  4. The body copies the pose and its muscles pull the physical arm and sword towards it.
//
// Nothing here is engine source. The two engine layouts we rely on (a tick function's small
// virtual table, and the skinned mesh's pair of bone transform arrays) are found and checked at
// run time; if a check fails motion arms switch off and the game runs as before.

#include "arm_control.hpp"

#include "config.hpp"
#include <mutex>
#include "log.hpp"
#include "stereo_device.hpp"
#include "ue_abi.hpp"
#include "vr_present.hpp"
#include "vrmath.hpp"

#include <Unreal/FProperty.hpp>
#include <Unreal/NameTypes.hpp>
#include <Unreal/Property/FStructProperty.hpp>
#include <Unreal/UClass.hpp>
#include <Unreal/UFunction.hpp>
#include <Unreal/UObject.hpp>
#include <Unreal/UObjectGlobals.hpp>
#include <Unreal/UScriptStruct.hpp>

#include <windows.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <map>
#include <mutex>
#include <string>
#include <vector>

using namespace RC;
using namespace RC::Unreal;

namespace hsvr
{
    namespace
    {
        // ---- small maths -----------------------------------------------------------------------
        double dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
        double length(const Vec3& v) { return std::sqrt(dot(v, v)); }
        Vec3 normalized(const Vec3& v, const Vec3& fallback = {1, 0, 0})
        {
            double n = length(v);
            return n > 1e-9 ? v * (1.0 / n) : fallback;
        }
        Vec3 perpendicular_part(const Vec3& v, const Vec3& axis_unit) { return v - axis_unit * dot(v, axis_unit); }

        Quat quat_axis_angle(const Vec3& axis_unit, double radians)
        {
            double s = std::sin(radians * 0.5);
            return {axis_unit.x * s, axis_unit.y * s, axis_unit.z * s, std::cos(radians * 0.5)};
        }

        // Rotation whose local X goes to `x` and local Y (made square to x) goes to `y`.
        Quat quat_from_frame(const Vec3& x_in, const Vec3& y_in)
        {
            Vec3 x = normalized(x_in);
            Vec3 y = normalized(perpendicular_part(y_in, x), std::fabs(x.z) < 0.9 ? normalized(cross({0, 0, 1}, x)) : Vec3{0, 1, 0});
            Vec3 z = cross(x, y);
            // Columns x, y, z.
            double m00 = x.x, m01 = y.x, m02 = z.x;
            double m10 = x.y, m11 = y.y, m12 = z.y;
            double m20 = x.z, m21 = y.z, m22 = z.z;
            double trace = m00 + m11 + m22;
            Quat q;
            if (trace > 0.0)
            {
                double s = 0.5 / std::sqrt(trace + 1.0);
                q = {(m21 - m12) * s, (m02 - m20) * s, (m10 - m01) * s, 0.25 / s};
            }
            else if (m00 > m11 && m00 > m22)
            {
                double s = 2.0 * std::sqrt(1.0 + m00 - m11 - m22);
                q = {0.25 * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s};
            }
            else if (m11 > m22)
            {
                double s = 2.0 * std::sqrt(1.0 + m11 - m00 - m22);
                q = {(m01 + m10) / s, 0.25 * s, (m12 + m21) / s, (m02 - m20) / s};
            }
            else
            {
                double s = 2.0 * std::sqrt(1.0 + m22 - m00 - m11);
                q = {(m02 + m20) / s, (m12 + m21) / s, 0.25 * s, (m10 - m01) / s};
            }
            return q.normalized();
        }

        Quat quat_from_rot_deg(double pitch, double yaw, double roll)
        {
            // Same order as Unreal's FRotator::Quaternion(): yaw, then pitch, then roll (intrinsic).
            Quat qy = quat_axis_angle({0, 0, 1}, yaw * kDegToRad);
            Quat qp = quat_axis_angle({0, 1, 0}, -pitch * kDegToRad);
            Quat qr = quat_axis_angle({1, 0, 0}, -roll * kDegToRad);
            return (qy * qp * qr).normalized();
        }

        double quat_angle_deg(const Quat& a, const Quat& b)
        {
            double d = std::fabs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w);
            return 2.0 * std::acos(std::min(1.0, d)) * kRadToDeg;
        }

        // A bone or component transform, scale left out (the character's bones are not scaled).
        struct Xf
        {
            Quat q;
            Vec3 t;
        };
        // `local` placed in `parent`'s frame (Unreal's local * parent).
        Xf compose(const Xf& local, const Xf& parent) { return {(parent.q * local.q).normalized(), parent.q.rotate(local.t) + parent.t}; }
        // `child` expressed in `parent`'s frame.
        Xf relative(const Xf& child, const Xf& parent)
        {
            Quat inv = parent.q.inverse();
            return {(inv * child.q).normalized(), inv.rotate(child.t - parent.t)};
        }

        // The engine's FTransform (double precision, vectorised): rotation x y z w, translation
        // x y z (+pad), scale x y z (+pad). 96 bytes; checked against a function's parameter size.
        struct alignas(16) RawXf
        {
            double q[4];
            double t[4];
            double s[4];
        };
        static_assert(sizeof(RawXf) == 96, "FTransform is 96 bytes in UE 5.4");
        Xf from_raw(const RawXf& r) { return {Quat{r.q[0], r.q[1], r.q[2], r.q[3]}.normalized(), Vec3{r.t[0], r.t[1], r.t[2]}}; }
        void to_raw(const Xf& x, RawXf& r)
        {
            r.q[0] = x.q.x;
            r.q[1] = x.q.y;
            r.q[2] = x.q.z;
            r.q[3] = x.q.w;
            r.t[0] = x.t.x;
            r.t[1] = x.t.y;
            r.t[2] = x.t.z;
        }

        // ---- reflection helpers (game thread) --------------------------------------------------
        UObject* object_property(UObject* owner, const TCHAR* name)
        {
            if (!owner) return nullptr;
            UObject** value = owner->GetValuePtrByPropertyNameInChain<UObject*>(name);
            return value ? *value : nullptr;
        }

        FName make_name(const wchar_t* text) { return FName(text, FNAME_Add); }

        struct alignas(16) SocketXfParams
        {
            FName InSocketName;
            uint8_t TransformSpace;
            uint8_t pad[7];
            RawXf ReturnValue;
        };
        static_assert(offsetof(SocketXfParams, ReturnValue) == 16, "layout");
        enum : uint8_t
        {
            RTS_World = 0,
            RTS_Component = 2
        };

        struct alignas(16) XfReturnParams
        {
            RawXf ReturnValue;
        };
        struct VectorReturnParams
        {
            ue::FVector ReturnValue;
        };
        struct RotatorReturnParams
        {
            ue::FRotator ReturnValue;
        };
        struct NameIntParams
        {
            FName BoneName;
            int32_t ReturnValue;
        };
        struct NameNameParams
        {
            FName BoneName;
            FName ReturnValue;
        };
        struct IntNameParams
        {
            int32_t BoneIndex;
            FName ReturnValue;
        };
        struct IntReturnParams
        {
            int32_t ReturnValue;
        };

        bool socket_transform(UObject* mesh, FName bone, uint8_t space, Xf& out)
        {
            if (!mesh) return false;
            UFunction* fn = mesh->GetFunctionByNameInChain(STR("GetSocketTransform"));
            if (!fn) return false;
            static bool size_checked = false, size_ok = false;
            if (!size_checked)
            {
                size_checked = true;
                size_ok = fn->GetParmsSize() == (int32_t)sizeof(SocketXfParams);
                if (!size_ok) HSVR_LOG("Arms: GetSocketTransform takes %d bytes, expected %d; motion arms stay off", fn->GetParmsSize(), (int)sizeof(SocketXfParams));
            }
            if (!size_ok) return false;
            SocketXfParams p{};
            p.InSocketName = bone;
            p.TransformSpace = space;
            mesh->ProcessEvent(fn, &p);
            out = from_raw(p.ReturnValue);
            return true;
        }

        bool component_location(UObject* component, Vec3& out)
        {
            if (!component) return false;
            UFunction* fn = component->GetFunctionByNameInChain(STR("K2_GetComponentLocation"));
            if (!fn) return false;
            VectorReturnParams p{};
            component->ProcessEvent(fn, &p);
            out = {p.ReturnValue.X, p.ReturnValue.Y, p.ReturnValue.Z};
            return true;
        }

        bool actor_location(UObject* actor, Vec3& out)
        {
            if (!actor) return false;
            UFunction* fn = actor->GetFunctionByNameInChain(STR("K2_GetActorLocation"));
            if (!fn) return false;
            VectorReturnParams p{};
            actor->ProcessEvent(fn, &p);
            out = {p.ReturnValue.X, p.ReturnValue.Y, p.ReturnValue.Z};
            return true;
        }

        bool actor_yaw(UObject* actor, double& yaw)
        {
            if (!actor) return false;
            UFunction* fn = actor->GetFunctionByNameInChain(STR("K2_GetActorRotation"));
            if (!fn) return false;
            RotatorReturnParams p{};
            actor->ProcessEvent(fn, &p);
            yaw = p.ReturnValue.Yaw;
            return true;
        }

        int32_t call_int(UObject* obj, const TCHAR* fn_name)
        {
            UFunction* fn = obj ? obj->GetFunctionByNameInChain(fn_name) : nullptr;
            if (!fn) return -1;
            IntReturnParams p{};
            obj->ProcessEvent(fn, &p);
            return p.ReturnValue;
        }

        // Reads memory that may not be there (used only while looking for engine layouts).
        bool safe_copy(void* dst, const void* src, size_t size)
        {
            __try
            {
                memcpy(dst, src, size);
                return true;
            }
            __except (EXCEPTION_EXECUTE_HANDLER)
            {
                return false;
            }
        }

        // ---- the driver skeleton's bone transforms ---------------------------------------------
        // USkinnedMeshComponent keeps `TArray<FTransform> ComponentSpaceTransformsArray[2]`, then
        // two more arrays, a float, `int32 CurrentEditableComponentTransforms` and
        // `int32 CurrentReadComponentTransforms` (UE 5.4.4 SkinnedMeshComponent.h). None is
        // reflected, so the pair of arrays is found by its shape and checked against the bone
        // transforms the engine itself reports.
        struct RawArray
        {
            RawXf* data;
            int32_t num;
            int32_t max;
        };
        constexpr size_t kReadIndexAfterArrays = 72;

        struct DriverLayout
        {
            bool searched = false;
            size_t arrays_offset = 0; // 0 = not found
        };
        DriverLayout g_layout;

        bool transforms_close(const Xf& a, const Xf& b)
        {
            return length(a.t - b.t) < 0.05 && quat_angle_deg(a.q, b.q) < 0.5;
        }

        bool find_driver_layout(UObject* driver, int32_t bone_count, const std::vector<FName>& check_bones, const std::vector<int32_t>& check_index)
        {
            auto* base = reinterpret_cast<uint8_t*>(driver);
            UClass* cls = driver->GetClassPrivate();
            int32_t object_size = cls ? cls->GetPropertiesSize() : 0x1000;
            if (object_size <= 0 || object_size > 0x10000) object_size = 0x1000;
            std::vector<Xf> reported;
            for (size_t i = 0; i < check_bones.size(); ++i)
            {
                Xf x;
                if (!socket_transform(driver, check_bones[i], RTS_Component, x)) return false;
                reported.push_back(x);
            }
            for (size_t off = 0x200; off + kReadIndexAfterArrays + 8 <= (size_t)object_size; off += 8)
            {
                RawArray arrays[2];
                if (!safe_copy(arrays, base + off, sizeof(arrays))) continue;
                // The first array always holds the bones; the second too when the mesh keeps two
                // copies (the default), else it is empty.
                if (arrays[0].num != bone_count || arrays[0].max < bone_count || !arrays[0].data) continue;
                bool second_full = arrays[1].num == bone_count && arrays[1].max >= bone_count && arrays[1].data && arrays[1].data != arrays[0].data;
                bool second_empty = arrays[1].num == 0 && arrays[1].max >= 0;
                if (!second_full && !second_empty) continue;
                int32_t indices[2];
                if (!safe_copy(indices, base + off + kReadIndexAfterArrays - 4, sizeof(indices))) continue;
                int32_t read = indices[1];
                if (read != 0 && read != 1) continue;
                if (read == 1 && !second_full) continue;
                bool all_match = true;
                for (size_t i = 0; i < check_bones.size() && all_match; ++i)
                {
                    RawXf raw;
                    if (!safe_copy(&raw, arrays[read].data + check_index[i], sizeof(raw))) all_match = false;
                    else if (!transforms_close(from_raw(raw), reported[i])) all_match = false;
                }
                if (!all_match) continue;
                g_layout.arrays_offset = off;
                HSVR_LOG("Arms: driver skeleton bone transforms found at offset 0x%zX (read buffer %d, editable %d, %d bones)", off, read, indices[0], bone_count);
                return true;
            }
            return false;
        }

        RawXf* driver_read_buffer(UObject* driver, int32_t bone_count)
        {
            if (!g_layout.arrays_offset) return nullptr;
            auto* base = reinterpret_cast<uint8_t*>(driver) + g_layout.arrays_offset;
            auto* arrays = reinterpret_cast<RawArray*>(base);
            int32_t read = *reinterpret_cast<int32_t*>(base + kReadIndexAfterArrays);
            if (read != 0 && read != 1) return nullptr;
            if (arrays[read].num != bone_count || !arrays[read].data) return nullptr;
            return arrays[read].data;
        }

        // ---- per-side state ----------------------------------------------------------------------
        struct Side
        {
            const wchar_t* suffix;
            const char* label;
            double out_sign; // +1 right, -1 left
            int controller_index;
            // Driver bone indices.
            int32_t clavicle = -1, upper = -1, lower = -1, hand = -1, parent = -1;
            FName parent_name, hand_name, middle_name, thumb_name, lower_name;
            // Learned from the game's own animation (bone-local).
            bool have_axes = false;
            Vec3 upper_axis, lower_axis; // bone direction in the bone's own frame
            bool have_hinge = false;
            Vec3 hinge_upper, hinge_lower; // elbow hinge axis in each bone's frame
            // How the hand holds things, learned per state (hand-local, from the physical body):
            // with a weapon, with a closed fist, and with an open hand. The game moves the hand
            // a lot between those states; one shared measurement made the wrist roll wildly when
            // the trigger was released and the game went to fists.
            struct GripFrame
            {
                Vec3 forward{1, 0, 0}; // handle direction (towards the thumb / the blade)
                Vec3 center{};         // from the hand bone to the middle of the fist / the grip
                bool have = false;
            };
            GripFrame grip_weapon, grip_closed, grip_open;
            // This frame's target.
            bool target_valid = false;
            Xf target_world; // where the hand bone should be
            Vec3 blade_world;
            // For the log.
            double last_error_cm = 0, last_error_deg = 0;
            double last_reach = 0;    // controller distance from the shoulder / arm length
            double elbow_angle = 0;   // how far the elbow is swung from its natural place, degrees
            double elbow_time = 0;
            double shoulder_shift = 0, shoulder_time = 0;
            double last_twist = 0;
            Vec3 assist_pos, assist_rot; // hold assist, in the upper body's frame (cm, rotation vector)
            double assist_time = 0;    // hand roll about the forearm last frame, radians (unwrapped)
            double reach_distance = -1.0; // smoothed distance the FABRIK goal is placed at
            double reach_time = 0.0;
        };

        struct State
        {
            bool enabled_by_key = true;
            bool failed = false;
            DWORD game_thread = 0;
            uint64_t frame = 0;
            UObject* pawn = nullptr;
            UObject* body = nullptr;   // CharacterMesh0
            UObject* driver = nullptr; // DriverSkeleton
            int32_t bone_count = 0;
            std::vector<int32_t> parents;
            bool ready = false; // bones known, layout found
            int32_t head_bone = -1;
            Side side[2] = {{L"_l", "left", -1.0, 0}, {L"_r", "right", 1.0, 1}};
            XrHandRaw hands[2];
            bool hands_valid = false;
            double actor_yaw = 0;
            bool menu_up = true;
            // What we last wrote, to tell whether the driver has animated since.
            uint64_t written_frame = 0;
            RawXf written_marker{};
            int32_t marker_bone = -1;
            std::vector<RawXf> animated; // the driver's own pose from its latest fresh frame
            bool have_animated = false;
            // Counters for the log.
            uint64_t fresh_count = 0, stale_count = 0, applied_count = 0;
            std::map<void*, void*> hooked; // tick function object -> its original vtable
        };
        State g_state;
        std::mutex g_hook_mutex;

        // ---- hooking the meshes' tick functions ------------------------------------------------
        // FTickFunction has four virtual functions in UE 5.4.4 (EngineBaseTypes.h): destructor,
        // ExecuteTick, DiagnosticMessage, DiagnosticContext. Each hooked tick function gets its
        // vtable pointer swapped for a copy whose ExecuteTick runs our code first; every other
        // object of the type is left alone.
        using ExecuteTickFn = void (*)(void* self, float delta, int tick_type, int current_thread, const void* completion);
        // Four are copied plus a margin (copying a few words past the table is harmless; only the
        // four are ever called).
        constexpr int kTickVtableSlots = 8;
        constexpr int kExecuteTickSlot = 1;
        std::map<void**, void**> g_fake_vtables; // original -> our copy (never freed)
        ExecuteTickFn g_original_execute = nullptr;

        void before_body_update();

        void hooked_execute_tick(void* self, float delta, int tick_type, int current_thread, const void* completion)
        {
            if (GetCurrentThreadId() == g_state.game_thread) before_body_update();
            g_original_execute(self, delta, tick_type, current_thread, completion);
        }

        bool in_game_module(void* address)
        {
            HMODULE module = nullptr;
            if (!GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT, (LPCWSTR)address, &module)) return false;
            return module == GetModuleHandleW(nullptr);
        }

        bool hook_component_tick(UObject* component)
        {
            FProperty* property = component->GetPropertyByNameInChain(STR("PrimaryComponentTick"));
            if (!property) return false;
            void* tick = reinterpret_cast<uint8_t*>(component) + property->GetOffset_Internal();
            void** vtable = *reinterpret_cast<void***>(tick);
            for (auto& entry : g_fake_vtables)
            {
                if (entry.second == vtable) return true; // already ours
            }
            if (!in_game_module(vtable[kExecuteTickSlot]))
            {
                HSVR_LOG("Arms: the tick function of %ls does not look like the engine's; not hooked", component->GetName().c_str());
                return false;
            }
            void** fake = nullptr;
            auto found = g_fake_vtables.find(vtable);
            if (found != g_fake_vtables.end())
            {
                fake = found->second;
            }
            else
            {
                auto original = reinterpret_cast<ExecuteTickFn>(vtable[kExecuteTickSlot]);
                if (g_original_execute && g_original_execute != original)
                {
                    HSVR_LOG("Arms: a second kind of tick function (%ls); not hooked", component->GetName().c_str());
                    return false;
                }
                g_original_execute = original;
                fake = new void*[kTickVtableSlots];
                for (int i = 0; i < kTickVtableSlots; ++i) fake[i] = vtable[i];
                fake[kExecuteTickSlot] = reinterpret_cast<void*>(&hooked_execute_tick);
                g_fake_vtables[vtable] = fake;
            }
            *reinterpret_cast<void***>(tick) = fake;
            return true;
        }

        // ---- finding the character's parts -------------------------------------------------------
        FName side_name(const wchar_t* stem, const Side& side)
        {
            std::wstring text = std::wstring(stem) + side.suffix;
            return make_name(text.c_str());
        }

        int32_t driver_bone_index(FName name)
        {
            UFunction* fn = g_state.driver->GetFunctionByNameInChain(STR("GetBoneIndex"));
            if (!fn) return -1;
            NameIntParams p{};
            p.BoneName = name;
            p.ReturnValue = -1;
            g_state.driver->ProcessEvent(fn, &p);
            return p.ReturnValue;
        }

        bool setup_pawn(UObject* pawn)
        {
            State& s = g_state;
            s.ready = false;
            s.have_animated = false;
            s.pawn = pawn;
            s.body = object_property(pawn, STR("Mesh"));
            s.driver = object_property(pawn, STR("DriverSkeleton"));
            if (!s.driver)
            {
                std::vector<UObject*> meshes;
                UObjectGlobals::FindAllOf(STR("SkeletalMeshComponent"), meshes);
                for (UObject* m : meshes)
                {
                    if (m && m->GetOuterPrivate() == pawn && m->GetName() == STR("DriverSkeleton")) s.driver = m;
                }
            }
            if (!s.body || !s.driver)
            {
                static UObject* told = nullptr;
                if (told == pawn) return false;
                told = pawn;
                HSVR_LOG("Arms: %ls has no %s; motion arms wait for a fighting character", pawn->GetName().c_str(), s.body ? "DriverSkeleton" : "body mesh");
                return false;
            }
            s.bone_count = call_int(s.driver, STR("GetNumBones"));
            if (s.bone_count <= 0 || s.bone_count > 1000) return false;
            // Parent of every driver bone (bones come parents first).
            s.parents.assign(s.bone_count, -1);
            UFunction* name_fn = s.driver->GetFunctionByNameInChain(STR("GetBoneName"));
            UFunction* parent_fn = s.driver->GetFunctionByNameInChain(STR("GetParentBone"));
            if (!name_fn || !parent_fn) return false;
            std::wstring names;
            for (int32_t i = 0; i < s.bone_count; ++i)
            {
                IntNameParams n{};
                n.BoneIndex = i;
                s.driver->ProcessEvent(name_fn, &n);
                NameNameParams p{};
                p.BoneName = n.ReturnValue;
                s.driver->ProcessEvent(parent_fn, &p);
                s.parents[i] = driver_bone_index(p.ReturnValue);
                names += n.ReturnValue.ToString() + L" ";
            }
            HSVR_LOG("Arms: driver skeleton %ls, %d bones: %ls", s.driver->GetFullName().c_str(), s.bone_count, names.c_str());
            for (int32_t i = 0; i < s.bone_count; ++i)
            {
                if (s.parents[i] >= i)
                {
                    HSVR_LOG("Arms: driver bone %d comes before its parent %d; motion arms stay off", i, s.parents[i]);
                    return false;
                }
            }
            std::vector<FName> check_names;
            std::vector<int32_t> check_index;
            for (Side& side : s.side)
            {
                side.clavicle = driver_bone_index(side_name(L"clavicle", side));
                side.upper = driver_bone_index(side_name(L"upperarm", side));
                side.lower = driver_bone_index(side_name(L"lowerarm", side));
                side.hand = driver_bone_index(side_name(L"hand", side));
                side.hand_name = side_name(L"hand", side);
                side.lower_name = side_name(L"lowerarm", side);
                side.middle_name = side_name(L"middle_01", side);
                side.thumb_name = side_name(L"thumb_01", side);
                side.have_axes = side.have_hinge = false;
                side.grip_weapon = side.grip_closed = side.grip_open = {};
                if (side.upper < 0 || side.lower < 0 || side.hand < 0 || s.parents[side.lower] != side.upper || s.parents[side.hand] != side.lower)
                {
                    HSVR_LOG("Arms: the driver's %s arm is not upperarm -> lowerarm -> hand (%d %d %d); motion arms stay off", side.label, side.upper, side.lower,
                             side.hand);
                    return false;
                }
                // The frame the arm hangs from: the clavicle's parent (upper spine), or the
                // upper arm's parent when there is no clavicle.
                side.parent = side.clavicle >= 0 ? s.parents[side.clavicle] : s.parents[side.upper];
                if (side.parent < 0) return false;
                IntNameParams n{};
                n.BoneIndex = side.parent;
                s.driver->ProcessEvent(name_fn, &n);
                side.parent_name = n.ReturnValue;
                check_names.push_back(side.hand_name);
                check_index.push_back(side.hand);
                check_names.push_back(side_name(L"upperarm", side));
                check_index.push_back(side.upper);
            }
            s.marker_bone = s.side[1].hand;
            if (!g_layout.arrays_offset && !find_driver_layout(s.driver, s.bone_count, check_names, check_index))
            {
                static int complaints = 0;
                if (++complaints <= 3) HSVR_LOG("Arms: could not find the driver skeleton's bone transforms (try %d); will try again", complaints);
                if (complaints >= 30)
                {
                    s.failed = true;
                    HSVR_LOG("Arms: giving up; motion arms stay off for this run");
                }
                return false;
            }
            // Hook the tick of every other skeletal mesh on the character, so that whichever
            // updates first after the driver has animated applies the arms.
            std::vector<UObject*> meshes;
            UObjectGlobals::FindAllOf(STR("SkeletalMeshComponent"), meshes);
            int hooked = 0;
            {
                std::lock_guard<std::mutex> lock(g_hook_mutex);
                for (UObject* m : meshes)
                {
                    if (!m || m == s.driver || m->GetOuterPrivate() != pawn) continue;
                    if (hook_component_tick(m)) ++hooked;
                }
            }
            if (hooked == 0 || !g_original_execute)
            {
                HSVR_LOG("Arms: no mesh of the character could be hooked; motion arms stay off");
                return false;
            }
            s.head_bone = driver_bone_index(FName(STR("Head"), FNAME_Add));
            HSVR_LOG("Arms: head bone %s (%d)", s.head_bone >= 0 ? "found" : "missing", s.head_bone);
            HSVR_LOG("Arms: ready on %ls (%d meshes follow the arms; arm frames %ls / %ls)", pawn->GetName().c_str(), hooked,
                     s.side[0].parent_name.ToString().c_str(), s.side[1].parent_name.ToString().c_str());
            s.ready = true;
            return true;
        }

        // ---- learning from the game's animation ------------------------------------------------
        void learn_from_animation(Side& side, const RawXf* pose)
        {
            Xf su = from_raw(pose[side.upper]), sl = from_raw(pose[side.lower]), sh = from_raw(pose[side.hand]);
            Vec3 u = sl.t - su.t, f = sh.t - sl.t;
            if (length(u) < 1.0 || length(f) < 1.0) return;
            Vec3 upper_axis = normalized(su.q.inverse().rotate(u));
            Vec3 lower_axis = normalized(sl.q.inverse().rotate(f));
            if (!side.have_axes)
            {
                side.upper_axis = upper_axis;
                side.lower_axis = lower_axis;
                side.have_axes = true;
            }
            Vec3 bend = cross(normalized(u), normalized(f));
            double sin_bend = length(bend);
            if (sin_bend < 0.25) return; // nearly straight: the hinge cannot be told
            Vec3 hinge = bend * (1.0 / sin_bend);
            Vec3 hu = normalized(su.q.inverse().rotate(hinge));
            Vec3 hl = normalized(sl.q.inverse().rotate(hinge));
            if (!side.have_hinge)
            {
                side.hinge_upper = hu;
                side.hinge_lower = hl;
                side.have_hinge = true;
                HSVR_LOG("Arms: %s elbow hinge learned from the game's animation (bend %.0f deg): upper (%.2f %.2f %.2f) lower (%.2f %.2f %.2f), "
                         "bone axes upper (%.2f %.2f %.2f) lower (%.2f %.2f %.2f)",
                         side.label, std::asin(std::min(1.0, sin_bend)) * kRadToDeg, hu.x, hu.y, hu.z, hl.x, hl.y, hl.z, upper_axis.x, upper_axis.y,
                         upper_axis.z, lower_axis.x, lower_axis.y, lower_axis.z);
            }
            else
            {
                // A slow average; an elbow is a fixed hinge, so this only takes out noise.
                side.hinge_upper = normalized(side.hinge_upper * 0.98 + hu * 0.02);
                side.hinge_lower = normalized(side.hinge_lower * 0.98 + hl * 0.02);
            }
        }

        // How the hand holds things, measured on the physical body. A weapon, a closed fist and an
        // open hand each get their own frame, and the frame for the current state is used.
        void learn_grip(Side& side, bool closed)
        {
            State& s = g_state;
            Xf hand;
            if (!socket_transform(s.body, side.hand_name, RTS_World, hand)) return;
            Quat inv = hand.q.inverse();
            Config cfg = config();
            bool from_weapon = false;
            Vec3 forward{1, 0, 0}, center{};
            if (side.out_sign > 0.0)
            {
                UObject* weapon = object_property(s.pawn, STR("Weapon R"));
                Vec3 root, head;
                if (weapon && actor_location(weapon, root) && component_location(object_property(weapon, STR("Head")), head) && length(head - root) > 3.0 &&
                    length(root - hand.t) < 40.0)
                {
                    forward = normalized(inv.rotate(head - root));
                    center = inv.rotate(root - hand.t) + forward * cfg.grip_shift_cm;
                    from_weapon = length(perpendicular_part(center, forward)) > 1.0;
                }
            }
            Side::GripFrame* frame = from_weapon ? &side.grip_weapon : (closed ? &side.grip_closed : &side.grip_open);
            if (!from_weapon)
            {
                Xf middle, thumb;
                if (!socket_transform(s.body, side.middle_name, RTS_World, middle) || !socket_transform(s.body, side.thumb_name, RTS_World, thumb)) return;
                Vec3 knuckle = inv.rotate(middle.t - hand.t);
                Vec3 thumb_dir = inv.rotate(thumb.t - hand.t);
                if (length(knuckle) < 2.0) return;
                Vec3 k = normalized(knuckle);
                forward = normalized(perpendicular_part(thumb_dir, k));
                center = knuckle * 0.7 + forward * cfg.grip_shift_cm;
            }
            if (!frame->have)
            {
                frame->forward = forward;
                frame->center = center;
                frame->have = true;
                HSVR_LOG("Arms: %s hand grip measured from %s: handle (%.2f %.2f %.2f), grip centre (%.1f %.1f %.1f) cm in the hand's frame", side.label,
                         from_weapon ? "the weapon in it" : (closed ? "its closed fist" : "its open hand"), forward.x, forward.y, forward.z, center.x, center.y,
                         center.z);
            }
            else
            {
                frame->forward = normalized(frame->forward * 0.9 + forward * 0.1);
                frame->center = frame->center * 0.9 + center * 0.1;
            }
        }

        // ---- where the hands should be (game thread, before the world ticks) -------------------
        void compute_targets()
        {
            State& s = g_state;
            Config cfg = config();
            Quat extra = quat_from_rot_deg(cfg.hand_pitch, cfg.hand_yaw, cfg.hand_roll);
            for (Side& side : s.side)
            {
                side.target_valid = false;
                const XrHandRaw& raw = s.hands[side.controller_index];
                bool wanted = side.out_sign > 0 ? cfg.arm_right : cfg.arm_left;
                bool closed = raw.trigger > 0.5f || raw.squeeze > 0.5f;
                const Side::GripFrame* frame = nullptr;
                if (side.grip_weapon.have) frame = &side.grip_weapon;
                else if (closed && side.grip_closed.have) frame = &side.grip_closed;
                else if (!closed && side.grip_open.have) frame = &side.grip_open;
                else if (side.grip_closed.have) frame = &side.grip_closed;
                else if (side.grip_open.have) frame = &side.grip_open;
                if (!wanted || !raw.valid || !frame)
                {
                    side.assist_pos = {};
                    side.assist_rot = {};
                    side.assist_time = 0;
                    continue;
                }
                Vec3 grip_pos;
                Quat grip_rot;
                if (!stereo_tracking_to_world(raw.px, raw.py, raw.pz, raw.qx, raw.qy, raw.qz, raw.qw, grip_pos, grip_rot)) continue;
                grip_rot = (grip_rot * extra).normalized();
                // The controller's handle (+X) and the direction from the wrist into the fist (-Z)
                // match the hand's handle and grip-centre directions for the current hand state.
                Quat hand_frame = quat_from_frame(frame->forward, frame->center);
                Quat controller_frame = quat_from_frame({1, 0, 0}, {0, 0, -1});
                Quat hand_rot = (grip_rot * controller_frame * hand_frame.inverse()).normalized();
                side.target_world.q = hand_rot;
                side.target_world.t = grip_pos - hand_rot.rotate(frame->center);
                side.blade_world = grip_rot.rotate({1, 0, 0});
                side.target_valid = true;
            }
        }

        void sync_aim()
        {
            State& s = g_state;
            Config cfg = config();
            if (!cfg.aim_sync) return;
            const Side& right = s.side[1];
            if (!right.target_valid) return;
            static FProperty* property = nullptr;
            static UClass* property_class = nullptr;
            static bool ok = false;
            UClass* cls = s.pawn->GetClassPrivate();
            if (cls != property_class)
            {
                property_class = cls;
                property = s.pawn->GetPropertyByNameInChain(STR("Aim Vector"));
                ok = false;
                if (property)
                {
                    // Seen live as a 2-D point; a 3-D vector (X and Y used) is accepted too.
                    auto* sp = static_cast<FStructProperty*>(property);
                    UScriptStruct* st = property->GetClass().GetName() == STR("StructProperty") ? sp->GetStruct() : nullptr;
                    std::wstring type = st ? std::wstring(st->GetName()) : std::wstring(property->GetClass().GetName());
                    ok = st && ((type == L"Vector2D" && property->GetSize() == 16) || (type == L"Vector" && property->GetSize() == 24));
                    HSVR_LOG("Arms: the game's aim (Aim Vector) is a %ls of %d bytes at 0x%X; %s", type.c_str(), property->GetSize(), property->GetOffset_Internal(),
                             ok ? "it follows the sword" : "not a type the mod can write; aim_sync does nothing");
                }
                else
                {
                    std::wstring names;
                    for (UStruct* st = cls; st; st = st->GetSuperStruct())
                    {
                        for (FProperty* p : st->ForEachProperty())
                        {
                            std::wstring n = p->GetName();
                            if (n.find(L"Aim") != std::wstring::npos) names += n + L" (" + std::wstring(p->GetClass().GetName()) + L") ";
                        }
                    }
                    HSVR_LOG("Arms: the character has no property named Aim Vector; aim_sync does nothing. Aim properties: %ls", names.c_str());
                }
            }
            if (!ok) return;
            Vec3 b = right.blade_world;
            double yaw = normalize_axis(std::atan2(b.y, b.x) * kRadToDeg - s.actor_yaw);
            double pitch = std::asin(std::max(-1.0, std::min(1.0, b.z))) * kRadToDeg;
            auto* aim = reinterpret_cast<ue::FVector2D*>(reinterpret_cast<uint8_t*>(s.pawn) + property->GetOffset_Internal());
            aim->X = std::max(-90.0, std::min(90.0, yaw * cfg.aim_scale));
            double y = -pitch * cfg.aim_scale * (cfg.aim_invert_y ? -1.0 : 1.0);
            aim->Y = std::max(-90.0, std::min(90.0, y));
        }

        // ---- bending the driver's arms (game thread, inside a mesh's tick) ---------------------
        // ---- general pose placement -------------------------------------------------------------
        // Puts one bone exactly at an asked component-space position and shares the displacement
        // across its ancestors by weight; optionally also sets its orientation and shares a part
        // of that rotation with the ancestors (the forearm rolling with the wrist).
        //
        // Chain 0 is arm-like (hand -> lowerarm -> upperarm -> clavicle; the spine joins only on a
        // long reach so the shoulder does not come off the body). Chain 1 is spine-like (even
        // weights, always including one more ancestor).
        // Smallest rotation that takes unit vector `from` to unit vector `to`.
        Quat rotation_between(const Vec3& from, const Vec3& to)
        {
            Vec3 a = normalized(from), b = normalized(to);
            double d = dot(a, b);
            if (d > 1.0 - 1e-9) return {};
            if (d < -1.0 + 1e-9)
            {
                Vec3 axis = normalized(cross(a, {0, 0, 1}));
                if (length(axis) < 1e-6) axis = normalized(cross(a, {0, 1, 0}));
                return quat_axis_angle(axis, 3.14159265358979);
            }
            Vec3 axis = cross(a, b);
            double s = length(axis);
            return quat_axis_angle(axis * (1.0 / s), std::atan2(s, d));
        }

        // FABRIK on any driver chain: fixed segment lengths, so the pose is one the physical
        // ragdoll can reproduce. Used for the hands (arm + shoulder + spine) and for the head
        // (neck + spine). `gentle_above` is a bone in the chain above which the solved rotation is
        // only partly taken (so the torso helps without swinging the whole body).
        void fabrik_chain(State& s, std::vector<Xf>& pose, std::vector<char>& changed, const RawXf* anim, int32_t leaf, const Vec3& target,
                          const Quat* leaf_rotation, int wanted_bones, int32_t gentle_above, double& smoothed_distance, double& smooth_time,
                          const Config& cfg)
        {
            if (leaf < 0 || leaf >= s.bone_count) return;
            int32_t bones[6];
            int n = 0;
            bones[n++] = leaf;
            int32_t node = s.parents[leaf];
            int wanted = std::max(2, std::min(6, wanted_bones));
            while (node >= 0 && n < wanted)
            {
                bones[n++] = node;
                node = s.parents[node];
            }
            if (n < 3) return;
            for (int i = 0; i < n / 2; ++i)
            {
                int32_t swap = bones[i];
                bones[i] = bones[n - 1 - i];
                bones[n - 1 - i] = swap;
            }
            int gentle_index = -1;
            if (gentle_above >= 0)
            {
                for (int i = 0; i < n; ++i)
                {
                    if (bones[i] == gentle_above)
                    {
                        gentle_index = i;
                        break;
                    }
                }
            }
            Vec3 p[6];
            for (int i = 0; i < n; ++i) p[i] = pose[bones[i]].t;
            double lengths[5]{};
            double total = 0.0;
            for (int i = 0; i < n - 1; ++i)
            {
                lengths[i] = length(p[i + 1] - p[i]);
                if (lengths[i] < 0.5) return;
                total += lengths[i];
            }
            Vec3 root = p[0];
            Vec3 goal = target;
            double distance = length(goal - root);
            if (distance <= 1e-4) return;

            double max_reach = total * std::max(0.5, std::min(1.0, cfg.pose_reach_max));
            double knee = total * std::max(0.3, std::min(1.0, cfg.pose_reach_knee));
            if (knee > max_reach) knee = max_reach;
            // The player's arms may be longer than the character's: scaling the requested distance
            // stops a fully outstretched player hand mapping far past the character's reach.
            distance *= std::max(0.5, std::min(1.5, cfg.pose_arm_scale));
            // How far past the soft knee this target is (0 = within reach, 1 = at/after the limit).
            double overreach = std::max(0.0, std::min(1.0, (distance - knee) / std::max(1.0, max_reach - knee)));
            double wanted_distance = distance;
            if (distance > knee)
            {
                double span = std::max(0.5, max_reach - knee);
                wanted_distance = knee + span * (1.0 - std::exp(-(distance - knee) / span));
            }
            double now = now_seconds();
            double dt = smooth_time > 0.0 ? std::min(0.1, now - smooth_time) : 0.0;
            smooth_time = now;
            if (smoothed_distance < 0.0 || cfg.pose_reach_smoothing <= 0.0) smoothed_distance = wanted_distance;
            else smoothed_distance += (wanted_distance - smoothed_distance) * (1.0 - std::exp(-dt / cfg.pose_reach_smoothing));
            Vec3 direction = normalized(goal - root);
            goal = root + direction * std::max(0.0, smoothed_distance);

            for (int iteration = 0; iteration < 10; ++iteration)
            {
                p[n - 1] = goal;
                for (int i = n - 2; i >= 0; --i) p[i] = p[i + 1] + normalized(p[i] - p[i + 1]) * lengths[i];
                p[0] = root;
                for (int i = 0; i < n - 1; ++i) p[i + 1] = p[i] + normalized(p[i + 1] - p[i]) * lengths[i];
            }
            for (int i = 0; i < n - 1; ++i)
            {
                Vec3 old_direction = from_raw(anim[bones[i + 1]]).t - from_raw(anim[bones[i]]).t;
                Vec3 new_direction = p[i + 1] - p[i];
                Quat delta = rotation_between(old_direction, new_direction);
                if (gentle_index >= 0 && i < gentle_index && i < 2)
                {
                    if (delta.w < 0.0) delta = {-delta.x, -delta.y, -delta.z, -delta.w};
                    double angle = 2.0 * std::acos(std::min(1.0, delta.w));
                    if (angle > 1e-4)
                    {
                        Vec3 axis = normalized(Vec3{delta.x, delta.y, delta.z}, {1, 0, 0});
                        // The more the target exceeds reach, the more the torso leans in, so the
                        // shoulder/chest help instead of the hand simply falling behind.
                        double share = std::max(0.0, cfg.pose_reach_spine_share);
                        share = share + (1.0 - share) * overreach * overreach;
                        delta = quat_axis_angle(axis, angle * std::min(1.0, share));
                    }
                }
                pose[bones[i]].q = (delta * pose[bones[i]].q).normalized();
                pose[bones[i]].t = p[i];
                changed[bones[i]] = 1;
            }
            pose[bones[n - 1]].t = p[n - 1];
            if (leaf_rotation) pose[bones[n - 1]].q = *leaf_rotation;
            changed[bones[n - 1]] = 1;
        }

        void fabrik_solve(State& s, Side& side, const RawXf* anim, std::vector<Xf>& pose, std::vector<char>& changed, const Vec3& target,
                          const Quat* hand_rotation, const Config& cfg)
        {
            fabrik_chain(s, pose, changed, anim, side.hand, target, hand_rotation, cfg.pose_reach_bones, side.clavicle, side.reach_distance,
                         side.reach_time, cfg);
        }

        // The skeletal mesh component's world transform, for drawing its bones in the world.
        bool component_to_world(UObject* component, Xf& out)
        {
            if (!component) return false;
            UFunction* fn = component->GetFunctionByNameInChain(STR("K2_GetComponentToWorld"));
            if (!fn) return false;
            struct Params
            {
                RawXf ReturnValue;
            } params{};
            if (fn->GetParmsSize() != (int32_t)sizeof(Params)) return false;
            component->ProcessEvent(fn, &params);
            out = from_raw(params.ReturnValue);
            return true;
        }

        void solve_side(Side& side, const RawXf* anim, std::vector<Xf>& pose, std::vector<char>& changed, const Config& cfg)
        {
            State& s = g_state;
            if (!side.target_valid || !side.have_axes) return;
            Xf parent_world;
            if (!socket_transform(s.body, side.parent_name, RTS_World, parent_world)) return;
            Xf parent_cs = pose[side.parent];
            // The target in the frame of the physical upper body, placed on the driver's upper body.
            Xf target_rel = relative(side.target_world, parent_world);
            // Hold assist: the muscles leave the real hand short of where it is sent by a steady
            // amount (sword weight, muscle strength). That steady part is measured in the upper
            // body's frame and added to what is sent, slowly (assist_seconds), within assist_max_cm
            // and assist_max_deg. Fast swings are not changed by it.
            Xf body_hand_now;
            if (cfg.assist_seconds > 0.0 && socket_transform(s.body, side.hand_name, RTS_World, body_hand_now))
            {
                Xf real_rel = relative(body_hand_now, parent_world);
                double now = now_seconds();
                double dt = side.assist_time > 0 ? std::min(0.1, now - side.assist_time) : 0.0;
                side.assist_time = now;
                double k = dt / cfg.assist_seconds;
                Vec3 pos_err = target_rel.t - real_rel.t;
                Quat rot_err = (target_rel.q * real_rel.q.inverse()).normalized();
                if (rot_err.w < 0) rot_err = {-rot_err.x, -rot_err.y, -rot_err.z, -rot_err.w};
                double half = std::acos(std::min(1.0, rot_err.w));
                double sn = std::sin(half);
                Vec3 rot_vec = sn > 1e-9 ? Vec3{rot_err.x, rot_err.y, rot_err.z} * (2.0 * half / sn) : Vec3{};
                side.assist_pos = side.assist_pos + pos_err * k;
                side.assist_rot = side.assist_rot + rot_vec * k;
                double max_pos = cfg.assist_max_cm, max_rot = cfg.assist_max_deg * kDegToRad;
                if (length(side.assist_pos) > max_pos) side.assist_pos = normalized(side.assist_pos) * max_pos;
                if (length(side.assist_rot) > max_rot) side.assist_rot = normalized(side.assist_rot) * max_rot;
                target_rel.t = target_rel.t + side.assist_pos;
                double ang = length(side.assist_rot);
                if (ang > 1e-6) target_rel.q = (quat_axis_angle(side.assist_rot * (1.0 / ang), ang) * target_rel.q).normalized();
            }
            else
            {
                side.assist_pos = {};
                side.assist_rot = {};
                side.assist_time = 0;
            }
            Xf target = compose(target_rel, parent_cs);
            if (cfg.pose_system)
            {
                fabrik_solve(s, side, anim, pose, changed, target.t, cfg.pose_hand_rotation ? &target.q : nullptr, cfg);
                static bool reported = false;
                if (!reported)
                {
                    reported = true;
                    HSVR_LOG("Arms: hand targets use the FABRIK chain solver (pose_system=1)");
                }
                return;
            }
            auto to_driver = [&](const Vec3& world_dir) { return parent_cs.q.rotate(parent_world.q.inverse().rotate(world_dir)); };

            Xf upper0 = pose[side.upper], lower0 = pose[side.lower], hand0 = pose[side.hand];
            double a = length(lower0.t - upper0.t), b = length(hand0.t - lower0.t);
            if (a < 1.0 || b < 1.0) return;
            Vec3 S = upper0.t;
            // Shoulder reach: like a real shoulder blade, the clavicle swings the shoulder towards a
            // hand that is beyond easy reach, by up to shoulder_reach_cm. The character's arms are
            // shorter than most players', so this lets the in-game hand get closer to the real one.
            if (side.clavicle >= 0 && cfg.shoulder_reach_cm > 0.0)
            {
                Xf clav = pose[side.clavicle];
                Vec3 bone = S - clav.t;
                double len = length(bone);
                double excess = length(target.t - S) - cfg.reach_knee * (a + b);
                double want = std::max(0.0, std::min(cfg.shoulder_reach_cm, excess));
                double now = now_seconds();
                double dt = side.shoulder_time > 0 ? std::min(0.1, now - side.shoulder_time) : 0.0;
                side.shoulder_time = now;
                double alpha = dt > 0 ? 1.0 - std::exp(-dt / 0.08) : 1.0;
                side.shoulder_shift += (want - side.shoulder_shift) * alpha;
                if (len > 1.0 && side.shoulder_shift > 0.05)
                {
                    Vec3 moved = bone + normalized(target.t - S) * side.shoulder_shift;
                    Vec3 from = bone * (1.0 / len), to = normalized(moved);
                    Vec3 cr = cross(from, to);
                    double sn = length(cr), cs = dot(from, to);
                    if (sn > 1e-9)
                    {
                        Quat turn = quat_axis_angle(cr * (1.0 / sn), std::atan2(sn, cs));
                        clav.q = (turn * clav.q).normalized();
                        pose[side.clavicle] = clav;
                        changed[side.clavicle] = 1;
                        S = clav.t + turn.rotate(bone);
                    }
                }
            }
            Vec3 to_target = target.t - S;
            double d = length(to_target);
            Vec3 dir = normalized(to_target, normalized(lower0.t - S));
            // Reach: a straight arm leaves the blade angle to the wrist alone, and the game's wrist
            // stops at about 78 degrees of bend. So the elbow is always kept a little bent: past
            // reach_knee of the full arm length the distance is eased towards reach_max.
            side.last_reach = d / (a + b);
            {
                double knee = std::max(0.3, std::min(0.98, cfg.reach_knee));
                double top = std::max(knee + 0.01, std::min(0.999, cfg.reach_max));
                double r = d / (a + b);
                if (r > knee) r = knee + (top - knee) * std::tanh((r - knee) / (top - knee));
                d = r * (a + b);
            }
            d = std::max(std::fabs(a - b) + 0.5, std::min(d, (a + b) * 0.999));

            // Elbow direction: down, out and back relative to the character's facing.
            Quat facing = quat_from_yaw_deg(s.actor_yaw);
            Vec3 pole_world = Vec3{0, 0, -cfg.elbow_down} + facing.rotate({-cfg.elbow_back, side.out_sign * cfg.elbow_out, 0});
            Vec3 pole = perpendicular_part(to_driver(pole_world), dir);
            if (length(pole) < 1e-3) pole = perpendicular_part(lower0.t - S, dir);
            Vec3 perp = normalized(pole, normalized(perpendicular_part({0, 0, -1}, dir)));
            double cos_a = std::max(-1.0, std::min(1.0, (a * a + d * d - b * b) / (2.0 * a * d)));
            double sin_a = std::sqrt(std::max(0.0, 1.0 - cos_a * cos_a));
            // The elbow's place around the shoulder-to-hand line decides which way the forearm
            // points and how it is rolled (the elbow is a hinge). Choose it so that the wrist needs
            // to bend less than wrist_bend_limit and roll less than twist_limit (the game's wrist
            // stops near 78 degrees of bend and 90 of roll), keeping it near its natural place and
            // near last frame's place otherwise.
            Vec3 side_axis = cross(dir, perp);
            Vec3 axis = side.lower_axis;
            Vec3 hand_dir = target.q.rotate(axis); // the hand's own lengthwise axis
            auto twist_of = [&](const Quat& lower_untwisted) {
                Quat rel = lower_untwisted.inverse() * target.q;
                double t = 2.0 * std::atan2(rel.x * axis.x + rel.y * axis.y + rel.z * axis.z, rel.w);
                if (t > kPi) t -= 2.0 * kPi;
                if (t < -kPi) t += 2.0 * kPi;
                return t;
            };
            auto arm_for = [&](double angle_deg, Vec3& elbow, Vec3& fore, Vec3& hinge_out, Quat& lower_untwisted) {
                double rad = angle_deg * kDegToRad;
                Vec3 p = normalized(perp * std::cos(rad) + side_axis * std::sin(rad));
                elbow = S + dir * (a * cos_a) + p * (a * sin_a);
                fore = normalized(S + dir * d - elbow);
                hinge_out = normalized(cross(p, dir));
                lower_untwisted = side.have_hinge ? (quat_from_frame(fore, hinge_out) * quat_from_frame(side.lower_axis, side.hinge_lower).inverse()).normalized() : Quat{};
            };
            if (side.have_hinge)
            {
                double best_cost = 1e30, best_angle = 0;
                double range = std::max(0.0, std::min(150.0, cfg.elbow_swing_max));
                for (double angle = -range; angle <= range + 1e-6; angle += 5.0)
                {
                    Vec3 e, fore, h;
                    Quat lu;
                    arm_for(angle, e, fore, h, lu);
                    double bend = std::acos(std::max(-1.0, std::min(1.0, dot(fore, hand_dir)))) * kRadToDeg;
                    double roll = std::fabs(twist_of(lu)) * kRadToDeg;
                    double over_bend = std::max(0.0, bend - cfg.wrist_bend_limit);
                    double over_roll = std::max(0.0, roll - cfg.twist_limit);
                    double moved = angle - side.elbow_angle;
                    double cost = over_bend * over_bend + over_roll * over_roll + cfg.elbow_stiffness * angle * angle + 0.01 * moved * moved;
                    if (cost < best_cost)
                    {
                        best_cost = cost;
                        best_angle = angle;
                    }
                }
                double now = now_seconds();
                double dt = side.elbow_time > 0 ? std::min(0.1, now - side.elbow_time) : 0.0;
                side.elbow_time = now;
                double alpha = dt > 0 ? 1.0 - std::exp(-dt / 0.06) : 1.0;
                side.elbow_angle += (best_angle - side.elbow_angle) * alpha;
            }
            Vec3 E, f, hinge;
            Quat lower_untwisted;
            arm_for(side.have_hinge ? side.elbow_angle : 0.0, E, f, hinge, lower_untwisted);
            Vec3 T = S + dir * d;
            Vec3 u = normalized(E - S);

            Quat upper_q, lower_q;
            Quat hand_q = target.q;
            if (side.have_hinge)
            {
                upper_q = quat_from_frame(u, hinge) * quat_from_frame(side.upper_axis, side.hinge_upper).inverse();
                // Hand roll about the forearm: kept continuous from frame to frame (no jump from
                // +180 to -180) and held within twist_clamp, so the arm never flips over.
                double twist = twist_of(lower_untwisted);
                while (twist - side.last_twist > kPi) twist -= 2.0 * kPi;
                while (twist - side.last_twist < -kPi) twist += 2.0 * kPi;
                double clamp = cfg.twist_clamp * kDegToRad;
                double sent_twist = std::max(-clamp, std::min(clamp, twist));
                side.last_twist = sent_twist;
                Quat rel = lower_untwisted.inverse() * target.q;
                double proj = rel.x * axis.x + rel.y * axis.y + rel.z * axis.z;
                Quat rel_twist = Quat{axis.x * proj, axis.y * proj, axis.z * proj, rel.w}.normalized();
                Quat rel_swing = rel * rel_twist.inverse();
                hand_q = (lower_untwisted * rel_swing * quat_axis_angle(axis, sent_twist)).normalized();
                double share = std::max(0.0, std::min(1.0, cfg.forearm_twist_share));
                lower_q = (lower_untwisted * quat_axis_angle(axis, sent_twist * share)).normalized();
            }
            else
            {
                // Hinge not learned yet: swing the animated bones the shortest way.
                auto swing = [](const Vec3& from, const Vec3& to) {
                    Vec3 cr = cross(from, to);
                    double sn = length(cr), c = dot(from, to);
                    if (sn < 1e-9) return Quat{};
                    return quat_axis_angle(cr * (1.0 / sn), std::atan2(sn, c));
                };
                upper_q = swing(normalized(lower0.t - S), u) * upper0.q;
                Vec3 lower_dir_now = (upper_q * upper0.q.inverse()).rotate(normalized(hand0.t - lower0.t));
                lower_q = swing(lower_dir_now, f) * (upper_q * upper0.q.inverse()) * lower0.q;
            }
            upper_q = upper_q.normalized();
            lower_q = lower_q.normalized();

            pose[side.upper] = {upper_q, S};
            pose[side.lower] = {lower_q, E};
            pose[side.hand] = {hand_q, T};
            changed[side.upper] = changed[side.lower] = changed[side.hand] = 1;

            // How far the physical hand is from where it is sent (for the log).
            Xf body_hand;
            if (socket_transform(s.body, side.hand_name, RTS_World, body_hand))
            {
                side.last_error_cm = length(body_hand.t - side.target_world.t);
                side.last_error_deg = quat_angle_deg(body_hand.q, side.target_world.q);
            }
            (void)anim;
        }

        // ---- recorder: every frame for a few seconds, started with the right trigger ----------
        // Writes Mods\HalfSwordVR\arms_record_<n>.csv. Per arm: how far the physical hand is from
        // where it was sent, and the wrist (hand relative to forearm) as sent and as it really is,
        // split into bend (swing) and roll (twist about the forearm), to see whether the wrist stops
        // at a limit or the muscles are too slow.
        struct Recorder
        {
            bool active = false;
            double start = 0, until = 0, cooldown_until = 0;
            int number = 0;
            std::string text;
            Quat last_target[2];
            double last_time = 0;
        };
        Recorder g_rec;

        void swing_twist_deg(const Quat& rel, const Vec3& axis, double& swing, double& twist)
        {
            double tw = 2.0 * std::atan2(rel.x * axis.x + rel.y * axis.y + rel.z * axis.z, rel.w);
            if (tw > kPi) tw -= 2.0 * kPi;
            if (tw < -kPi) tw += 2.0 * kPi;
            twist = tw * kRadToDeg;
            Vec3 bent = rel.rotate(axis);
            swing = std::acos(std::max(-1.0, std::min(1.0, dot(bent, axis)))) * kRadToDeg;
        }

        void recorder_tick()
        {
            State& s = g_state;
            double now = now_seconds();
            if (!g_rec.active)
            {
                if (now >= g_rec.cooldown_until && s.hands[1].valid && s.hands[1].trigger > 0.7f && !s.menu_up && s.ready)
                {
                    g_rec.active = true;
                    g_rec.start = now;
                    g_rec.until = now + 8.0;
                    g_rec.number++;
                    g_rec.last_time = 0;
                    g_rec.text = "t,side,err_cm,err_deg,target_speed_dps,sent_wrist_bend,sent_wrist_roll,real_wrist_bend,real_wrist_roll,sent_elbow,real_elbow,reach,elbow_swing\n";
                    HSVR_LOG("Arms: recording 8 seconds (right trigger)");
                }
                return;
            }
            if (now < g_rec.until) return;
            g_rec.active = false;
            g_rec.cooldown_until = now + 2.0;
            char name[64];
            snprintf(name, sizeof(name), "arms_record_%d.csv", g_rec.number);
            std::wstring path = mod_dir() + std::wstring(name, name + strlen(name));
            if (FILE* f = _wfopen(path.c_str(), L"w"))
            {
                fputs(g_rec.text.c_str(), f);
                fclose(f);
            }
            HSVR_LOG("Arms: recording saved as %s", name);
        }

        void record_frame(const std::vector<Xf>& pose)
        {
            State& s = g_state;
            if (!g_rec.active) return;
            double now = now_seconds();
            double dt = g_rec.last_time > 0 ? now - g_rec.last_time : 0;
            for (int k = 0; k < 2; ++k)
            {
                Side& side = s.side[k];
                if (!side.target_valid || !side.have_axes) continue;
                Xf hand, lower, upper;
                if (!socket_transform(s.body, side.hand_name, RTS_World, hand) || !socket_transform(s.body, side.lower_name, RTS_World, lower) ||
                    !socket_transform(s.body, side_name(L"upperarm", side), RTS_World, upper))
                    continue;
                double speed = dt > 0 ? quat_angle_deg(g_rec.last_target[k], side.target_world.q) / dt : 0;
                g_rec.last_target[k] = side.target_world.q;
                double sb, sr, rb, rr, se, re, dummy;
                swing_twist_deg(pose[side.lower].q.inverse() * pose[side.hand].q, side.lower_axis, sb, sr);
                swing_twist_deg(lower.q.inverse() * hand.q, side.lower_axis, rb, rr);
                swing_twist_deg(pose[side.upper].q.inverse() * pose[side.lower].q, side.upper_axis, se, dummy);
                swing_twist_deg(upper.q.inverse() * lower.q, side.upper_axis, re, dummy);
                char line[256];
                snprintf(line, sizeof(line), "%.3f,%c,%.1f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.2f,%.0f\n", now - g_rec.start, k ? 'R' : 'L', length(hand.t - side.target_world.t),
                         quat_angle_deg(hand.q, side.target_world.q), speed, sb, sr, rb, rr, se, re, side.last_reach, side.elbow_angle);
                g_rec.text += line;
            }
            g_rec.last_time = now;
        }

        bool same_raw(const RawXf& a, const RawXf& b) { return memcmp(a.q, b.q, sizeof(a.q)) == 0 && memcmp(a.t, b.t, sizeof(a.t)) == 0; }

        void before_body_update()
        {
            State& s = g_state;
            if (!s.ready || s.failed || !s.enabled_by_key || s.menu_up || !s.hands_valid) return;
            if (!s.side[0].target_valid && !s.side[1].target_valid) return;
            Config cfg = config();
            if (!cfg.motion_arms) return;
            RawXf* buffer = driver_read_buffer(s.driver, s.bone_count);
            if (!buffer) return;
            bool stale = s.have_animated && s.marker_bone >= 0 && same_raw(buffer[s.marker_bone], s.written_marker);
            if (stale && s.written_frame == s.frame) return; // done already this frame
            if (!stale)
            {
                // The driver has animated since we last wrote: keep its own pose and learn from it.
                s.animated.assign(buffer, buffer + s.bone_count);
                s.have_animated = true;
                ++s.fresh_count;
                for (Side& side : s.side) learn_from_animation(side, s.animated.data());
            }
            else
            {
                ++s.stale_count;
            }
            std::vector<Xf> pose(s.bone_count);
            for (int32_t i = 0; i < s.bone_count; ++i) pose[i] = from_raw(s.animated[i]);
            std::vector<char> changed(s.bone_count, 0);
            // The head is a fixed point: the character's head is pulled to where the headset is
            // (in the recentered world frame) and the neck/spine solve to allow it. Solved before
            // the arms and then carried into them, so the shoulders stay attached to the spine.
            if (cfg.pose_head && s.head_bone >= 0)
            {
                Vec3 head_target;
                if (stereo_headset_world(head_target))
                {
                    static double head_reach_distance = -1.0;
                    static double head_reach_time = 0.0;
                    fabrik_chain(s, pose, changed, s.animated.data(), s.head_bone, head_target, nullptr, cfg.pose_head_bones, -1, head_reach_distance,
                                 head_reach_time, cfg);
                    for (int32_t i = 0; i < s.bone_count; ++i)
                    {
                        if (changed[i]) continue;
                        int32_t parent = s.parents[i];
                        if (parent < 0 || !changed[parent]) continue;
                        pose[i] = compose(relative(from_raw(s.animated[i]), from_raw(s.animated[parent])), pose[parent]);
                        changed[i] = 1;
                    }
                }
            }
            for (Side& side : s.side) solve_side(side, s.animated.data(), pose, changed, cfg);
            // Carry every bone below a moved one along with it (fingers, weapon bone).
            bool any = false;
            for (int32_t i = 0; i < s.bone_count; ++i)
            {
                if (changed[i])
                {
                    any = true;
                    continue;
                }
                int32_t p = s.parents[i];
                if (p < 0 || !changed[p]) continue;
                Xf local = relative(from_raw(s.animated[i]), from_raw(s.animated[p]));
                pose[i] = compose(local, pose[p]);
                changed[i] = 1;
            }
            if (!any) return;
            for (int32_t i = 0; i < s.bone_count; ++i)
            {
                if (changed[i]) to_raw(pose[i], buffer[i]);
            }
            s.written_marker = buffer[s.marker_bone];
            s.written_frame = s.frame;
            ++s.applied_count;
            record_frame(pose);
        }

        void log_status(const Config& cfg)
        {
            State& s = g_state;
            if (!cfg.log_arms) return;
            static double last = 0.0;
            double now = now_seconds();
            if (now - last < 1.0) return;
            last = now;
            const XrHandRaw& l = s.hands[0];
            const XrHandRaw& r = s.hands[1];
            HSVR_LOG("Arms: %s | controllers L %s R %s (trigger %.2f/%.2f grip %.2f/%.2f) | right hand %s err %.1f cm %.0f deg | left hand %s err %.1f cm %.0f deg | "
                     "driver frames fresh %llu late %llu applied %llu",
                     !cfg.motion_arms ? "off in settings" : (!s.enabled_by_key ? "off by key" : (s.menu_up ? "menu" : (s.ready ? "on" : "waiting"))),
                     l.valid ? "tracked" : "lost", r.valid ? "tracked" : "lost", l.trigger, r.trigger, l.squeeze, r.squeeze,
                     s.side[1].target_valid ? "sent" : "-", s.side[1].last_error_cm, s.side[1].last_error_deg, s.side[0].target_valid ? "sent" : "-",
                     s.side[0].last_error_cm, s.side[0].last_error_deg, (unsigned long long)s.fresh_count, (unsigned long long)s.stale_count,
                     (unsigned long long)s.applied_count);
        }
    } // namespace


    void arm_toggle()
    {
        g_state.enabled_by_key = !g_state.enabled_by_key;
        HSVR_LOG(g_state.enabled_by_key ? "Key: motion arms on" : "Key: motion arms off (the game's own arm animation)");
    }

    void arm_debug_draw()
    {
        Config cfg = config();
        if (!cfg.debug_draw) return;
        State& s = g_state;
        static int debug_tick = 0;
        if ((++debug_tick & 1) != 0) return;
        static bool reported = false;
        if (!s.ready || !s.pawn || !s.driver)
        {
            if (!reported)
            {
                reported = true;
                HSVR_LOG("DebugDraw: the driver skeleton is not set up yet (arms must have found a character once)");
            }
            return;
        }

        // Show the driver mesh itself: the game keeps this skeletal mesh hidden, and making it
        // visible is a native, safe way to see what the pose code is bending.
        static double last_visibility = 0.0;
        if (now_seconds() - last_visibility >= 1.0)
        {
            last_visibility = now_seconds();
            struct VisibilityParams
            {
                bool NewVisibility;
                bool PropagateToChildren;
            };
            if (UFunction* fn = s.driver->GetFunctionByNameInChain(STR("SetVisibility")))
            {
                VisibilityParams params{true, true};
                s.driver->ProcessEvent(fn, &params);
            }
            if (UFunction* fn = s.driver->GetFunctionByNameInChain(STR("SetHiddenInGame")))
            {
                VisibilityParams params{false, true};
                s.driver->ProcessEvent(fn, &params);
            }
        }
    }

    void arm_game_tick(UObject* pawn, bool menu_up)
    {
        State& s = g_state;
        s.game_thread = GetCurrentThreadId();
        ++s.frame;
        s.menu_up = menu_up;
        Config cfg = config();
        if (s.failed || (!cfg.motion_arms && !cfg.debug_draw))
        {
            log_status(cfg);
            return;
        }
        if (pawn != s.pawn)
        {
            s.ready = false;
            s.pawn = pawn;
            s.body = s.driver = nullptr;
            for (Side& side : s.side) side.target_valid = false;
            if (pawn) HSVR_LOG("Arms: new character %ls", pawn->GetFullName().c_str());
        }
        if (!pawn) return;
        if (!s.ready)
        {
            // Try now and then (a character that is still being set up, or the tavern's pawn).
            static double next_try = 0.0;
            double now = now_seconds();
            if (now < next_try) return;
            next_try = now + 2.0;
            if (!setup_pawn(pawn)) return;
        }
        s.hands_valid = vr_locate_hands(s.hands);
        actor_yaw(pawn, s.actor_yaw);
        // The grip is measured a few times a second (it changes when a weapon is picked up).
        static double next_grip = 0.0;
        double now = now_seconds();
        if (now >= next_grip)
        {
            next_grip = now + 0.25;
            for (Side& side : s.side)
            {
                bool closed = s.hands[side.controller_index].trigger > 0.5f || s.hands[side.controller_index].squeeze > 0.5f;
                learn_grip(side, closed);
            }
        }
        compute_targets();
        recorder_tick();
        if (s.enabled_by_key && !menu_up && s.hands_valid) sync_aim();
        log_status(cfg);
    }
} // namespace hsvr