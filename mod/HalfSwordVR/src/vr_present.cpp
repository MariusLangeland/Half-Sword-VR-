#include "vr_present.hpp"

#include "config.hpp"
#include "engine_slot.hpp"
#include "log.hpp"
#include "xr_loader.hpp"

#include <dxgi1_4.h>
#ifdef _MSC_VER
#include <intrin.h>
#define HSVR_RETURN_ADDRESS() _ReturnAddress()
#else
#define HSVR_RETURN_ADDRESS() __builtin_return_address(0)
#endif
#include <tlhelp32.h>

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <cstring>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace hsvr
{
    VrShared& shared()
    {
        static VrShared s;
        return s;
    }

    namespace
    {
        // Tiny COM smart pointer (avoids pulling in WRL so this file also builds with MinGW for checks).
        template <typename T>
        class Com
        {
          public:
            Com() = default;
            Com(const Com& o) : p(o.p)
            {
                if (p) p->AddRef();
            }
            Com(Com&& o) noexcept : p(o.p) { o.p = nullptr; }
            ~Com() { reset(); }
            Com& operator=(const Com& o)
            {
                if (this != &o)
                {
                    if (o.p) o.p->AddRef();
                    reset();
                    p = o.p;
                }
                return *this;
            }
            Com& operator=(Com&& o) noexcept
            {
                if (this != &o)
                {
                    reset();
                    p = o.p;
                    o.p = nullptr;
                }
                return *this;
            }
            void reset()
            {
                if (p)
                {
                    p->Release();
                    p = nullptr;
                }
            }
            void assign(T* raw)
            {
                if (raw) raw->AddRef();
                reset();
                p = raw;
            }
            T* get() const { return p; }
            T* operator->() const { return p; }
            T** put()
            {
                reset();
                return &p;
            }
            explicit operator bool() const { return p != nullptr; }

          private:
            T* p = nullptr;
        };

        // ---------------------------------------------------------------- frame pose pipeline --
        struct PresentItem
        {
            FramePose pose;
            int rect[2][4]{};
            bool rect_valid[2]{false, false};
            Com<ID3D12Resource> scene;
        };

        struct Pipeline
        {
            std::mutex mutex;
            std::deque<FramePose> game_queue; // game thread -> presenting thread
            bool rects_started = false;       // render thread: a view rectangle of this frame was seen
            int rect[2][4]{};
            bool rect_valid[2]{false, false};
            std::deque<PresentItem> present_queue; // render thread -> presenting thread (picture and rectangles)
            FramePose last_latched;
            bool separate_mode = false;
            uint64_t sequence = 0;
            uint64_t dropped_game = 0;
            uint64_t dropped_present = 0;
            uint64_t reused_render = 0;
            uint64_t dropped_stale = 0;
            // Since the last statistics line: how many Presents had finished between a pose being
            // taken on the game thread and its picture being presented (last slot: 4 or more).
            uint64_t age_count[5]{};
            size_t depth_min = 99, depth_max = 0; // poses waiting when a Present looked
        };
        Pipeline g_pipe;

        // Presents of the game's window that have completely finished.
        std::atomic<uint64_t> g_presents_done{0};
        thread_local bool t_game_present = false;

        // Called with g_pipe.mutex held, once per Present: the pose the picture now being
        // presented was drawn with.
        //
        // Poses and pictures are matched by order, and order alone can stay one step out for good
        // (a pose whose frame was never drawn waits in the queue for ever, and every later picture
        // then goes out with the pose of the frame before it: the world swims when the head turns).
        // The engine gives a hard limit that catches this: the game thread may not start frame N
        // before the render thread finished frame N-2, which itself waits for the Present of frame
        // N-3. So between taking the pose of a frame and the start of its Present at most
        // `max_age` (2) other Presents can finish. A pose older than that belongs to an earlier
        // picture and is thrown away.
        bool take_pose_for_present(FramePose& out, size_t allowed, int max_age)
        {
            uint64_t done = g_presents_done.load();
            size_t depth = g_pipe.game_queue.size();
            g_pipe.depth_min = std::min(g_pipe.depth_min, depth);
            g_pipe.depth_max = std::max(g_pipe.depth_max, depth);
            while (g_pipe.game_queue.size() > allowed)
            {
                g_pipe.game_queue.pop_front();
                ++g_pipe.dropped_game;
            }
            if (max_age >= 0)
            {
                // Keep the newest even if it looks old (a long stall): something must be shown.
                while (g_pipe.game_queue.size() > 1 && g_pipe.game_queue.front().presents_done + (uint64_t)max_age < done)
                {
                    g_pipe.game_queue.pop_front();
                    ++g_pipe.dropped_stale;
                }
            }
            if (g_pipe.game_queue.empty())
            {
                ++g_pipe.reused_render;
                return false;
            }
            out = g_pipe.game_queue.front();
            g_pipe.game_queue.pop_front();
            uint64_t age = done >= out.presents_done ? done - out.presents_done : 0;
            ++g_pipe.age_count[age > 4 ? 4 : age];
            return true;
        }

        // ------------------------------------------------------------------------- OpenXR ----
        XrApi xr;
        XrInstance g_instance = XR_NULL_HANDLE;
        XrSystemId g_system = XR_NULL_SYSTEM_ID;
        std::atomic<XrSession> g_session{XR_NULL_HANDLE};
        std::atomic<XrSpace> g_local_space{XR_NULL_HANDLE};
        std::atomic<int64_t> g_frame_time{0};
        std::atomic<int64_t> g_frame_period{11111111};
        std::atomic<int64_t> g_lead_ns{22222222};
        bool g_vr_available = false;
        bool g_vr_failed = false;
        bool g_fake_headset = false;

        // Pretend headset for tests without SteamVR: sits at the origin and looks slowly left and right.
        void fake_pose(FramePose& pose)
        {
            double t = now_seconds();
            Config cfg = config();
            // Held still at a chosen direction (for comparing pictures frame against frame), or
            // moving around that direction by the chosen amount.
            float amount = cfg.dev_fake_still ? 0.0f : (float)cfg.dev_fake_scale;
            const float to_radians = 3.14159265f / 180.0f;
            float yaw = (float)cfg.dev_fake_yaw * to_radians + (float)std::sin(t * 1.0) * 0.35f * amount;     // positive = left
            float pitch = (float)cfg.dev_fake_pitch * to_radians + (float)std::sin(t * 0.6) * 0.15f * amount; // positive = up
            // OpenXR orientation: yaw about +Y, then pitch about the head's +X.
            float cy = std::cos(yaw * 0.5f), sy = std::sin(yaw * 0.5f);
            float cp = std::cos(pitch * 0.5f), sp = std::sin(pitch * 0.5f);
            float qx = cy * sp, qy = sy * cp, qz = -sy * sp, qw = cy * cp;
            float sway = (float)std::sin(t * 0.4) * 0.10f * amount; // metres, side to side
            for (int e = 0; e < 2; ++e)
            {
                float side = e == 0 ? -0.032f : 0.032f;
                XrEyeRaw& eye = pose.eye[e];
                // Eye offset along the head's right axis (rotated by yaw only; good enough for a test).
                eye.px = sway + side * std::cos(yaw);
                eye.py = 0.0f;
                eye.pz = -side * std::sin(yaw);
                eye.qx = qx;
                eye.qy = qy;
                eye.qz = qz;
                eye.qw = qw;
                eye.angle_left = e == 0 ? -0.87f : -0.70f;
                eye.angle_right = e == 0 ? 0.70f : 0.87f;
                eye.angle_up = 0.79f;
                eye.angle_down = -0.79f;
            }
            pose.valid = true;
            pose.predicted_time = 0;
        }

        std::mutex g_recenter_mutex;
        Recenter g_recenter;
        float g_recenter_xr_pos[3]{0, 0, 0};
        float g_recenter_xr_yaw = 0.0f;

        const char* xr_result_name(XrResult r)
        {
            static thread_local char buffer[XR_MAX_RESULT_STRING_SIZE];
            if (xr.ResultToString && g_instance != XR_NULL_HANDLE && XR_SUCCEEDED(xr.ResultToString(g_instance, r, buffer))) return buffer;
            snprintf(buffer, sizeof(buffer), "XrResult %d", (int)r);
            return buffer;
        }

        bool process_running(const wchar_t* exe_name)
        {
            HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
            if (snap == INVALID_HANDLE_VALUE) return false;
            PROCESSENTRY32W entry{};
            entry.dwSize = sizeof(entry);
            bool found = false;
            if (Process32FirstW(snap, &entry))
            {
                do
                {
                    if (_wcsicmp(entry.szExeFile, exe_name) == 0)
                    {
                        found = true;
                        break;
                    }
                } while (Process32NextW(snap, &entry));
            }
            CloseHandle(snap);
            return found;
        }

        void copy_eye(XrEyeRaw& out, const XrView& view)
        {
            out.px = view.pose.position.x;
            out.py = view.pose.position.y;
            out.pz = view.pose.position.z;
            out.qx = view.pose.orientation.x;
            out.qy = view.pose.orientation.y;
            out.qz = view.pose.orientation.z;
            out.qw = view.pose.orientation.w;
            out.angle_left = view.fov.angleLeft;
            out.angle_right = view.fov.angleRight;
            out.angle_up = view.fov.angleUp;
            out.angle_down = view.fov.angleDown;
        }

        // Locates both eyes at `time`. Keeps the previous orientation/position where tracking is lost.
        bool locate_views(XrTime time, FramePose& io)
        {
            XrSession session = g_session.load();
            XrSpace space = g_local_space.load();
            if (session == XR_NULL_HANDLE || space == XR_NULL_HANDLE) return false;
            XrView views[2]{{XR_TYPE_VIEW}, {XR_TYPE_VIEW}};
            XrViewLocateInfo info{XR_TYPE_VIEW_LOCATE_INFO};
            info.viewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
            info.displayTime = time;
            info.space = space;
            XrViewState state{XR_TYPE_VIEW_STATE};
            uint32_t count = 0;
            XrResult r = xr.LocateViews(session, &info, &state, 2, &count, views);
            if (XR_FAILED(r) || count < 2)
            {
                HSVR_LOG_EVERY(5.0, "xrLocateViews failed: %s", xr_result_name(r));
                return false;
            }
            bool orientation_ok = (state.viewStateFlags & XR_VIEW_STATE_ORIENTATION_VALID_BIT) != 0;
            bool position_ok = (state.viewStateFlags & XR_VIEW_STATE_POSITION_VALID_BIT) != 0;
            for (int e = 0; e < 2; ++e)
            {
                XrEyeRaw fresh;
                copy_eye(fresh, views[e]);
                XrEyeRaw& dst = io.eye[e];
                if (!orientation_ok && io.valid)
                {
                    fresh.qx = dst.qx;
                    fresh.qy = dst.qy;
                    fresh.qz = dst.qz;
                    fresh.qw = dst.qw;
                }
                if (!position_ok && io.valid)
                {
                    fresh.px = dst.px;
                    fresh.py = dst.py;
                    fresh.pz = dst.pz;
                }
                dst = fresh;
            }
            if (!orientation_ok && !io.valid) return false;
            io.predicted_time = time;
            io.valid = true;
            return true;
        }

        // ---------------------------------------------------------------------- controllers ----
        // One action set with a grip pose, trigger and squeeze for each hand. Bindings are
        // suggested for the common controller types; SteamVR maps whichever controllers are in
        // use (the Steam Frame's included) onto one of them.
        struct HandActions
        {
            std::atomic<bool> ready{false};
            std::atomic<bool> profile_changed{true};
            XrActionSet set = XR_NULL_HANDLE;
            XrAction grip_pose = XR_NULL_HANDLE;
            XrAction trigger = XR_NULL_HANDLE;
            XrAction squeeze = XR_NULL_HANDLE;
            XrPath hand_path[2]{XR_NULL_PATH, XR_NULL_PATH};
            XrSpace grip_space[2]{XR_NULL_HANDLE, XR_NULL_HANDLE};
            XrHandRaw last[2];
        };
        HandActions g_hands;

        XrPath to_path(const char* text)
        {
            XrPath path = XR_NULL_PATH;
            xr.StringToPath(g_instance, text, &path);
            return path;
        }

        XrAction make_action(const char* name, const char* label, XrActionType type)
        {
            XrActionCreateInfo info{XR_TYPE_ACTION_CREATE_INFO};
            strncpy(info.actionName, name, XR_MAX_ACTION_NAME_SIZE - 1);
            strncpy(info.localizedActionName, label, XR_MAX_LOCALIZED_ACTION_NAME_SIZE - 1);
            info.actionType = type;
            info.countSubactionPaths = 2;
            info.subactionPaths = g_hands.hand_path;
            XrAction action = XR_NULL_HANDLE;
            XrResult r = xr.CreateAction(g_hands.set, &info, &action);
            if (XR_FAILED(r)) HSVR_LOG("OpenXR: could not make the controller action %s: %s", name, xr_result_name(r));
            return action;
        }

        // One call per interaction profile, with every binding the mod wants. It must be one
        // call: some runtimes replace (rather than merge) the suggestions for a profile, and a
        // second call then silently drops the hand pose and trigger bindings.
        void suggest_profile(const char* profile, const char* trigger_input, const char* squeeze_input)
        {
            std::vector<XrActionSuggestedBinding> bindings;
            std::vector<std::string> paths;
            paths.reserve(16);
            const char* sides[2] = {"left", "right"};
            auto add = [&](XrAction action, const char* side, const char* suffix) {
                if (!suffix || action == XR_NULL_HANDLE) return;
                paths.push_back(std::string("/user/hand/") + side + suffix);
                bindings.push_back({action, to_path(paths.back().c_str())});
            };
            for (int h = 0; h < 2; ++h)
            {
                add(g_hands.grip_pose, sides[h], "/input/grip/pose");
                add(g_hands.trigger, sides[h], trigger_input);
                add(g_hands.squeeze, sides[h], squeeze_input);
            }
            XrInteractionProfileSuggestedBinding suggested{XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING};
            suggested.interactionProfile = to_path(profile);
            suggested.countSuggestedBindings = (uint32_t)bindings.size();
            suggested.suggestedBindings = bindings.data();
            XrResult r = xr.SuggestInteractionProfileBindings(g_instance, &suggested);
            if (XR_FAILED(r)) HSVR_LOG("OpenXR: controller bindings for %s not accepted: %s", profile, xr_result_name(r));
        }

        // Called once, right after the session is made (bindings must be in before attaching).
        void create_hand_actions(XrSession session)
        {
            if (!xr.CreateActionSet || !xr.SyncActions)
            {
                HSVR_LOG("OpenXR: the runtime lacks the controller functions; no motion controls");
                return;
            }
            g_hands.hand_path[0] = to_path("/user/hand/left");
            g_hands.hand_path[1] = to_path("/user/hand/right");
            XrActionSetCreateInfo set_info{XR_TYPE_ACTION_SET_CREATE_INFO};
            strncpy(set_info.actionSetName, "halfsword", XR_MAX_ACTION_SET_NAME_SIZE - 1);
            strncpy(set_info.localizedActionSetName, "Half Sword", XR_MAX_LOCALIZED_ACTION_SET_NAME_SIZE - 1);
            XrResult r = xr.CreateActionSet(g_instance, &set_info, &g_hands.set);
            if (XR_FAILED(r))
            {
                HSVR_LOG("OpenXR: could not make the controller action set: %s", xr_result_name(r));
                return;
            }
            g_hands.grip_pose = make_action("hand_pose", "Hand pose", XR_ACTION_TYPE_POSE_INPUT);
            g_hands.trigger = make_action("trigger", "Trigger", XR_ACTION_TYPE_FLOAT_INPUT);
            g_hands.squeeze = make_action("squeeze", "Grip", XR_ACTION_TYPE_FLOAT_INPUT);
            if (g_hands.grip_pose == XR_NULL_HANDLE) return;
            suggest_profile("/interaction_profiles/valve/index_controller", "/input/trigger/value", "/input/squeeze/value");
            suggest_profile("/interaction_profiles/oculus/touch_controller", "/input/trigger/value", "/input/squeeze/value");
            suggest_profile("/interaction_profiles/htc/vive_controller", "/input/trigger/value", "/input/squeeze/click");
            suggest_profile("/interaction_profiles/khr/simple_controller", "/input/select/click", nullptr);
            XrSessionActionSetsAttachInfo attach{XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO};
            attach.countActionSets = 1;
            attach.actionSets = &g_hands.set;
            r = xr.AttachSessionActionSets(session, &attach);
            if (XR_FAILED(r))
            {
                HSVR_LOG("OpenXR: could not attach the controller actions: %s", xr_result_name(r));
                return;
            }
            for (int h = 0; h < 2; ++h)
            {
                XrActionSpaceCreateInfo space_info{XR_TYPE_ACTION_SPACE_CREATE_INFO};
                space_info.action = g_hands.grip_pose;
                space_info.subactionPath = g_hands.hand_path[h];
                space_info.poseInActionSpace.orientation.w = 1.0f;
                r = xr.CreateActionSpace(session, &space_info, &g_hands.grip_space[h]);
                if (XR_FAILED(r))
                {
                    HSVR_LOG("OpenXR: could not make the %s hand space: %s", h ? "right" : "left", xr_result_name(r));
                    return;
                }
            }
            g_hands.ready.store(true);
            HSVR_LOG("OpenXR: controller actions ready (hand poses, trigger, grip)");
        }

        float read_float_action(XrSession session, XrAction action, int hand)
        {
            if (action == XR_NULL_HANDLE) return 0.0f;
            XrActionStateGetInfo info{XR_TYPE_ACTION_STATE_GET_INFO};
            info.action = action;
            info.subactionPath = g_hands.hand_path[hand];
            XrActionStateFloat state{XR_TYPE_ACTION_STATE_FLOAT};
            if (XR_FAILED(xr.GetActionStateFloat(session, &info, &state)) || !state.isActive) return 0.0f;
            return state.currentState;
        }

        void log_interaction_profiles(XrSession session)
        {
            for (int h = 0; h < 2; ++h)
            {
                XrInteractionProfileState state{XR_TYPE_INTERACTION_PROFILE_STATE};
                if (XR_FAILED(xr.GetCurrentInteractionProfile(session, g_hands.hand_path[h], &state))) continue;
                char text[XR_MAX_PATH_LENGTH] = "none";
                uint32_t length = 0;
                if (state.interactionProfile != XR_NULL_PATH) xr.PathToString(g_instance, state.interactionProfile, sizeof(text), &length, text);
                HSVR_LOG("OpenXR: %s controller uses the bindings for %s", h ? "right" : "left", text);
            }
        }

        // --------------------------------------------------------------------------- D3D12 ----
        struct XrSwap
        {
            XrSwapchain handle = XR_NULL_HANDLE;
            int width = 0, height = 0;
            std::vector<XrSwapchainImageD3D12KHR> images;
            Com<ID3D12DescriptorHeap> rtv_heap;
        };

        struct FrameResources
        {
            Com<ID3D12CommandAllocator> allocator;
            UINT64 fence_value = 0;
        };

        constexpr int kRing = 8;
        constexpr int kSrvPerFrame = 2;

        struct Gfx
        {
            bool ready = false;
            IDXGISwapChain* game_swapchain = nullptr; // not owned; identity only
            Com<ID3D12Device> device;
            Com<ID3D12CommandQueue> queue;
            FrameResources ring[kRing];
            int ring_index = 0;
            Com<ID3D12GraphicsCommandList> cmd;
            Com<ID3D12Fence> fence;
            UINT64 fence_counter = 0;
            HANDLE fence_event = nullptr;
            Com<ID3D12RootSignature> root_signature;
            Com<ID3D12PipelineState> pipeline;
            // The same shader set up for drawing into the game window (monitor picture).
            Com<ID3DBlob> vs_blob, ps_blob;
            Com<ID3D12PipelineState> window_pipeline, window_blend_pipeline;
            DXGI_FORMAT window_format = DXGI_FORMAT_UNKNOWN;
            bool window_pipeline_failed = false;
            Com<ID3D12DescriptorHeap> window_rtv_heap; // slot 0: monitor picture, slot 1: clear, slot 2: clear of a stand-in
            Com<ID3D12DescriptorHeap> srv_heap;
            UINT srv_size = 0, rtv_size = 0;
            DXGI_FORMAT xr_format = DXGI_FORMAT_UNKNOWN;
            XrSwap eye[2];
            XrSwap ui;
            // Copy of the window's back buffer (flat UI) that we can sample.
            Com<ID3D12Resource> ui_copy;
            DXGI_FORMAT ui_copy_format = DXGI_FORMAT_UNKNOWN;
            UINT ui_copy_width = 0, ui_copy_height = 0;
            D3D12_RESOURCE_STATES ui_copy_state = D3D12_RESOURCE_STATE_COPY_DEST;
            // Session state.
            bool session_running = false;
            XrSessionState session_state = XR_SESSION_STATE_UNKNOWN;
            bool session_lost = false;
            bool frame_begun = false;
            bool should_render = false;
            XrTime frame_time = 0;
            PresentItem last_item;
            bool have_last_item = false;
            // Checks on the wipe of the window buffer (see clear_next_window_buffer).
            ID3D12Resource* wiped_buffer = nullptr; // identity only, never used as a reference
            UINT wiped_index = 0;
            uint64_t wipes = 0, wipe_mismatches = 0, wipe_checks = 0;
            bool swapchain_logged = false;
            ID3D12Resource* logged_scene = nullptr;
            uint64_t presents = 0;
        };
        Gfx g;
        std::mutex g_present_mutex;

        using PresentFn = HRESULT(STDMETHODCALLTYPE*)(IDXGISwapChain*, UINT, UINT);
        using Present1Fn = HRESULT(STDMETHODCALLTYPE*)(IDXGISwapChain1*, UINT, UINT, const DXGI_PRESENT_PARAMETERS*);
        PresentFn g_original_present = nullptr;
        Present1Fn g_original_present1 = nullptr;
        thread_local bool t_in_present = false;

        // A Direct3D 12 swap chain does not tell us which command queue it presents on, so we
        // watch the queues the game submits work to and pick the one that belongs to its device.
        using ExecuteFn = void(STDMETHODCALLTYPE*)(ID3D12CommandQueue*, UINT, ID3D12CommandList* const*);
        ExecuteFn g_original_execute = nullptr;
        std::atomic<bool> g_queue_search_over{false};
        std::mutex g_queue_mutex;
        std::vector<Com<ID3D12CommandQueue>> g_seen_queues;

        void STDMETHODCALLTYPE hooked_execute(ID3D12CommandQueue* self, UINT count, ID3D12CommandList* const* lists)
        {
            if (!g_queue_search_over.load(std::memory_order_relaxed) && self)
            {
                std::lock_guard<std::mutex> lock(g_queue_mutex);
                bool known = false;
                for (auto& q : g_seen_queues) known = known || q.get() == self;
                if (!known && g_seen_queues.size() < 16 && !g_queue_search_over.load())
                {
                    D3D12_COMMAND_QUEUE_DESC desc = self->GetDesc();
                    if (desc.Type == D3D12_COMMAND_LIST_TYPE_DIRECT)
                    {
                        Com<ID3D12CommandQueue> keep;
                        keep.assign(self);
                        g_seen_queues.push_back(std::move(keep));
                    }
                }
            }
            g_original_execute(self, count, lists);
        }

        bool same_object(IUnknown* a, IUnknown* b)
        {
            if (!a || !b) return false;
            Com<IUnknown> ia, ib;
            if (FAILED(a->QueryInterface(IID_PPV_ARGS(ia.put()))) || FAILED(b->QueryInterface(IID_PPV_ARGS(ib.put())))) return false;
            return ia.get() == ib.get();
        }

        // True if the swap chain object holds a pointer to this queue somewhere near its start.
        bool swapchain_refers_to(IDXGISwapChain* swapchain, ID3D12CommandQueue* queue)
        {
            const uint8_t* base = reinterpret_cast<const uint8_t*>(swapchain);
            for (size_t offset = 0; offset < 0x800; offset += sizeof(void*))
            {
                const uint8_t* p = base + offset;
                // Check each page once before reading from it.
                if ((offset == 0 || (reinterpret_cast<uintptr_t>(p) & 0xFFF) == 0) && !memory_readable(p, sizeof(void*))) return false;
                void* value = nullptr;
                memcpy(&value, p, sizeof(value));
                if (value == queue) return true;
            }
            return false;
        }

        // The game's graphics queue for this swap chain, or null if it has not submitted work yet.
        Com<ID3D12CommandQueue> find_game_queue(IDXGISwapChain* swapchain, ID3D12Device* device)
        {
            std::lock_guard<std::mutex> lock(g_queue_mutex);
            std::vector<ID3D12CommandQueue*> matches;
            for (auto& q : g_seen_queues)
            {
                Com<ID3D12Device> queue_device;
                if (SUCCEEDED(q->GetDevice(IID_PPV_ARGS(queue_device.put()))) && same_object(queue_device.get(), device)) matches.push_back(q.get());
            }
            Com<ID3D12CommandQueue> chosen;
            if (matches.empty()) return chosen;
            ID3D12CommandQueue* pick = matches[0];
            if (matches.size() > 1)
            {
                int referred = 0;
                for (ID3D12CommandQueue* q : matches)
                {
                    if (swapchain_refers_to(swapchain, q))
                    {
                        if (referred++ == 0) pick = q;
                    }
                }
                HSVR_LOG("D3D: the game uses %zu graphics queues; %d of them belong to the swap chain", matches.size(), referred);
            }
            chosen.assign(pick);
            g_queue_search_over.store(true);
            g_seen_queues.clear();
            return chosen;
        }

        const char* kShaderSource = R"(
cbuffer Constants : register(b0)
{
    float4 uvRect;   // xy = offset, zw = scale
    float4 params;   // x: decode sRGB, y: 1 = key black to transparent, 2 = same plus the source's alpha, z: alpha gain, w: minimum alpha (backdrop)
    float4 cursor;   // xy = uv (negative = none), z = aspect (w/h), w = radius (fraction of height)
};
Texture2D sourceTexture : register(t0);
SamplerState linearSampler : register(s0);
struct VSOut { float4 pos : SV_Position; float2 uv : TEXCOORD0; };
VSOut VSMain(uint id : SV_VertexID)
{
    VSOut o;
    float2 p = float2((id << 1) & 2, id & 2);
    o.uv = p;
    o.pos = float4(p * float2(2.0, -2.0) + float2(-1.0, 1.0), 0.0, 1.0);
    return o;
}
float3 SrgbToLinear(float3 c)
{
    float3 lo = c / 12.92;
    float3 hi = pow((c + 0.055) / 1.055, 2.4);
    return lerp(hi, lo, step(c, 0.04045));
}
float4 PSMain(VSOut i) : SV_Target
{
    float2 uv = uvRect.xy + i.uv * uvRect.zw;
    float4 texel = sourceTexture.SampleLevel(linearSampler, uv, 0);
    float3 rgb = saturate(texel.rgb);
    float a = 1.0;
    if (params.y > 0.5)
    {
        // The source is flat UI drawn over black, so its colour is already "colour times
        // opacity". Brightness is the best guess at opacity; the output stays premultiplied,
        // so faint glows stay faint instead of turning into solid colour.
        a = max(saturate(max(rgb.r, max(rgb.g, rgb.b)) * params.z), params.w);
        // The engine's UI pass also adds up how much of each pixel it covered in the alpha
        // channel (the buffer starts at 0). Using it keeps dark but solid parts of a menu
        // from turning see-through.
        if (params.y > 1.5) a = max(a, saturate(texel.a));
    }
    if (cursor.x >= 0.0)
    {
        float2 d = (i.uv - cursor.xy) * float2(cursor.z, 1.0);
        float r = length(d);
        if (r < cursor.w)
        {
            rgb = r < cursor.w * 0.6 ? float3(1.0, 1.0, 1.0) : float3(0.0, 0.0, 0.0);
            a = 1.0;
        }
    }
    if (params.x > 0.5)
    {
        rgb = SrgbToLinear(rgb);
    }
    return float4(rgb, a);
}
)";

        struct BlitConstants
        {
            float uv_rect[4];
            float params[4];
            float cursor[4];
        };

        DXGI_FORMAT sampling_format(DXGI_FORMAT f, bool& is_float)
        {
            is_float = false;
            switch (f)
            {
            case DXGI_FORMAT_R8G8B8A8_TYPELESS:
            case DXGI_FORMAT_R8G8B8A8_UNORM:
            case DXGI_FORMAT_R8G8B8A8_UNORM_SRGB:
                return DXGI_FORMAT_R8G8B8A8_UNORM;
            case DXGI_FORMAT_B8G8R8A8_TYPELESS:
            case DXGI_FORMAT_B8G8R8A8_UNORM:
            case DXGI_FORMAT_B8G8R8A8_UNORM_SRGB:
                return DXGI_FORMAT_B8G8R8A8_UNORM;
            case DXGI_FORMAT_B8G8R8X8_TYPELESS:
            case DXGI_FORMAT_B8G8R8X8_UNORM:
            case DXGI_FORMAT_B8G8R8X8_UNORM_SRGB:
                return DXGI_FORMAT_B8G8R8X8_UNORM;
            case DXGI_FORMAT_R10G10B10A2_TYPELESS:
            case DXGI_FORMAT_R10G10B10A2_UNORM:
                return DXGI_FORMAT_R10G10B10A2_UNORM;
            case DXGI_FORMAT_R16G16B16A16_TYPELESS:
            case DXGI_FORMAT_R16G16B16A16_FLOAT:
                is_float = true;
                return DXGI_FORMAT_R16G16B16A16_FLOAT;
            case DXGI_FORMAT_R11G11B10_FLOAT:
                is_float = true;
                return f;
            default:
                return f;
            }
        }

        bool is_srgb_format(DXGI_FORMAT f)
        {
            return f == DXGI_FORMAT_R8G8B8A8_UNORM_SRGB || f == DXGI_FORMAT_B8G8R8A8_UNORM_SRGB;
        }

        template <typename Fn>
        Fn module_function(const wchar_t* module_name, const char* function_name)
        {
            HMODULE module = GetModuleHandleW(module_name);
            if (!module) module = LoadLibraryW(module_name);
            if (!module) return nullptr;
            return reinterpret_cast<Fn>(reinterpret_cast<void*>(GetProcAddress(module, function_name)));
        }

        using D3DCompileFn = HRESULT(WINAPI*)(LPCVOID, SIZE_T, LPCSTR, const void*, void*, LPCSTR, LPCSTR, UINT, UINT, ID3DBlob**, ID3DBlob**);
        using SerializeRootSignatureFn = HRESULT(WINAPI*)(const D3D12_ROOT_SIGNATURE_DESC*, D3D_ROOT_SIGNATURE_VERSION, ID3DBlob**, ID3DBlob**);

        // The blit shader bound to one output format; `blend` lays the source over what is there.
        Com<ID3D12PipelineState> make_pipeline(DXGI_FORMAT target_format, bool blend)
        {
            Com<ID3D12PipelineState> pipeline;
            D3D12_GRAPHICS_PIPELINE_STATE_DESC pd{};
            pd.pRootSignature = g.root_signature.get();
            pd.VS = {g.vs_blob->GetBufferPointer(), g.vs_blob->GetBufferSize()};
            pd.PS = {g.ps_blob->GetBufferPointer(), g.ps_blob->GetBufferSize()};
            D3D12_RENDER_TARGET_BLEND_DESC& rt = pd.BlendState.RenderTarget[0];
            rt.RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL;
            if (blend)
            {
                rt.BlendEnable = TRUE;
                rt.SrcBlend = D3D12_BLEND_ONE; // the shader's output is premultiplied
                rt.DestBlend = D3D12_BLEND_INV_SRC_ALPHA;
                rt.BlendOp = D3D12_BLEND_OP_ADD;
                rt.SrcBlendAlpha = D3D12_BLEND_ONE;
                rt.DestBlendAlpha = D3D12_BLEND_INV_SRC_ALPHA;
                rt.BlendOpAlpha = D3D12_BLEND_OP_ADD;
            }
            pd.SampleMask = 0xFFFFFFFFu;
            pd.RasterizerState.FillMode = D3D12_FILL_MODE_SOLID;
            pd.RasterizerState.CullMode = D3D12_CULL_MODE_NONE;
            pd.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
            pd.NumRenderTargets = 1;
            pd.RTVFormats[0] = target_format;
            pd.SampleDesc.Count = 1;
            HRESULT hr = g.device->CreateGraphicsPipelineState(&pd, IID_PPV_ARGS(pipeline.put()));
            if (FAILED(hr)) HSVR_LOG("D3D: CreateGraphicsPipelineState (format %d) failed 0x%08X", (int)target_format, (unsigned)hr);
            return pipeline;
        }

        bool create_blit_pipeline()
        {
            auto compile = module_function<D3DCompileFn>(L"d3dcompiler_47.dll", "D3DCompile");
            auto serialize = module_function<SerializeRootSignatureFn>(L"d3d12.dll", "D3D12SerializeRootSignature");
            if (!compile || !serialize)
            {
                HSVR_LOG("D3D: shader compiler or root signature serializer not available");
                return false;
            }
            Com<ID3DBlob> vs, ps, errors;
            HRESULT hr = compile(kShaderSource, strlen(kShaderSource), "hsvr_blit", nullptr, nullptr, "VSMain", "vs_5_0", 0, 0, vs.put(), errors.put());
            if (FAILED(hr))
            {
                HSVR_LOG("D3D: vertex shader failed to compile: %s", errors ? (const char*)errors->GetBufferPointer() : "?");
                return false;
            }
            hr = compile(kShaderSource, strlen(kShaderSource), "hsvr_blit", nullptr, nullptr, "PSMain", "ps_5_0", 0, 0, ps.put(), errors.put());
            if (FAILED(hr))
            {
                HSVR_LOG("D3D: pixel shader failed to compile: %s", errors ? (const char*)errors->GetBufferPointer() : "?");
                return false;
            }

            D3D12_DESCRIPTOR_RANGE range{};
            range.RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_SRV;
            range.NumDescriptors = 1;
            range.BaseShaderRegister = 0;
            D3D12_ROOT_PARAMETER params[2]{};
            params[0].ParameterType = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE;
            params[0].DescriptorTable.NumDescriptorRanges = 1;
            params[0].DescriptorTable.pDescriptorRanges = &range;
            params[0].ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;
            params[1].ParameterType = D3D12_ROOT_PARAMETER_TYPE_32BIT_CONSTANTS;
            params[1].Constants.ShaderRegister = 0;
            params[1].Constants.Num32BitValues = sizeof(BlitConstants) / 4;
            params[1].ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;
            D3D12_STATIC_SAMPLER_DESC sampler{};
            sampler.Filter = D3D12_FILTER_MIN_MAG_MIP_LINEAR;
            sampler.AddressU = sampler.AddressV = sampler.AddressW = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
            sampler.MaxLOD = D3D12_FLOAT32_MAX;
            sampler.ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;
            D3D12_ROOT_SIGNATURE_DESC rsd{};
            rsd.NumParameters = 2;
            rsd.pParameters = params;
            rsd.NumStaticSamplers = 1;
            rsd.pStaticSamplers = &sampler;
            Com<ID3DBlob> rs_blob;
            hr = serialize(&rsd, D3D_ROOT_SIGNATURE_VERSION_1, rs_blob.put(), errors.put());
            if (FAILED(hr))
            {
                HSVR_LOG("D3D: root signature serialize failed 0x%08X", (unsigned)hr);
                return false;
            }
            hr = g.device->CreateRootSignature(0, rs_blob->GetBufferPointer(), rs_blob->GetBufferSize(), IID_PPV_ARGS(g.root_signature.put()));
            if (FAILED(hr))
            {
                HSVR_LOG("D3D: CreateRootSignature failed 0x%08X", (unsigned)hr);
                return false;
            }

            g.vs_blob = vs;
            g.ps_blob = ps;
            g.pipeline = make_pipeline(g.xr_format, false);
            return (bool)g.pipeline;
        }

        bool create_xr_swapchain(XrSwap& swap, int width, int height, const char* label)
        {
            XrSession session = g_session.load();
            XrSwapchainCreateInfo info{XR_TYPE_SWAPCHAIN_CREATE_INFO};
            info.usageFlags = XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT | XR_SWAPCHAIN_USAGE_SAMPLED_BIT;
            info.format = g.xr_format;
            info.sampleCount = 1;
            info.width = (uint32_t)width;
            info.height = (uint32_t)height;
            info.faceCount = 1;
            info.arraySize = 1;
            info.mipCount = 1;
            XrResult r = xr.CreateSwapchain(session, &info, &swap.handle);
            if (XR_FAILED(r))
            {
                HSVR_LOG("OpenXR: xrCreateSwapchain(%s %dx%d) failed: %s", label, width, height, xr_result_name(r));
                return false;
            }
            uint32_t count = 0;
            xr.EnumerateSwapchainImages(swap.handle, 0, &count, nullptr);
            swap.images.assign(count, {XR_TYPE_SWAPCHAIN_IMAGE_D3D12_KHR});
            r = xr.EnumerateSwapchainImages(swap.handle, count, &count, reinterpret_cast<XrSwapchainImageBaseHeader*>(swap.images.data()));
            if (XR_FAILED(r) || count == 0)
            {
                HSVR_LOG("OpenXR: xrEnumerateSwapchainImages(%s) failed: %s", label, xr_result_name(r));
                return false;
            }
            D3D12_DESCRIPTOR_HEAP_DESC hd{};
            hd.Type = D3D12_DESCRIPTOR_HEAP_TYPE_RTV;
            hd.NumDescriptors = count;
            if (FAILED(g.device->CreateDescriptorHeap(&hd, IID_PPV_ARGS(swap.rtv_heap.put())))) return false;
            for (uint32_t i = 0; i < count; ++i)
            {
                D3D12_RENDER_TARGET_VIEW_DESC rtv{};
                rtv.Format = g.xr_format;
                rtv.ViewDimension = D3D12_RTV_DIMENSION_TEXTURE2D;
                D3D12_CPU_DESCRIPTOR_HANDLE h = swap.rtv_heap->GetCPUDescriptorHandleForHeapStart();
                h.ptr += (SIZE_T)i * g.rtv_size;
                g.device->CreateRenderTargetView(swap.images[i].texture, &rtv, h);
            }
            swap.width = width;
            swap.height = height;
            HSVR_LOG("OpenXR: swapchain %s %dx%d, %u images", label, width, height, count);
            return true;
        }

        void idle_pump_thread();

        // First Present from the game: find its D3D12 queue and create the OpenXR session on it.
        // ---- stand-in window pictures ----------------------------------------------------------
        // Half Sword has AMD's frame generation built in, and that add-on gives the game its own
        // "stand-in" pictures to draw on in place of the window's buffers; at each Present it
        // copies the finished stand-in into the real window buffer. The game's menu pass never
        // clears what it draws on, so wiping the real buffer (below) is not enough: old menus and
        // old mouse pointers stay on the stand-ins wherever nothing new covers them. The stand-ins
        // cannot be asked for, so they are recognised by that copy: any whole-picture copy into a
        // window buffer names its source as a stand-in. They are used in turn, so once the order
        // is known the one that comes next is wiped as well.
        struct StandIn
        {
            Com<ID3D12Resource> resource;
            uint64_t last_seen = 0;
        };
        std::mutex g_standin_mutex;
        std::vector<StandIn> g_standins;              // in the order first seen, which is the order of use
        bool g_standin_cycle_known = false;           // a source has come round again
        ID3D12Resource* g_standin_latest = nullptr;   // source of the newest copy (identity only)
        uint64_t g_standin_copies = 0, g_standin_wipes = 0, g_standin_restarts = 0;
        std::atomic<ID3D12Resource*> g_window_buffers[8]; // the real window buffers (identity only)
        std::atomic<bool> g_standin_watch{false};

        using CopyResourceFn = void(STDMETHODCALLTYPE*)(ID3D12GraphicsCommandList*, ID3D12Resource*, ID3D12Resource*);
        using CopyTextureRegionFn = void(STDMETHODCALLTYPE*)(ID3D12GraphicsCommandList*, const D3D12_TEXTURE_COPY_LOCATION*, UINT, UINT, UINT,
                                                              const D3D12_TEXTURE_COPY_LOCATION*, const D3D12_BOX*);
        CopyResourceFn g_original_copy_resource = nullptr;
        CopyTextureRegionFn g_original_copy_region = nullptr;

        void note_copy_into_window(ID3D12Resource* dst, ID3D12Resource* src)
        {
            if (!dst || !src || dst == src || !g_standin_watch.load(std::memory_order_relaxed)) return;
            bool into_window = false, from_window = false;
            for (auto& buffer : g_window_buffers)
            {
                ID3D12Resource* b = buffer.load(std::memory_order_relaxed);
                into_window = into_window || b == dst;
                from_window = from_window || b == src;
            }
            if (!into_window || from_window) return;
            std::lock_guard<std::mutex> lock(g_standin_mutex);
            uint64_t now = g_presents_done.load();
            ++g_standin_copies;
            for (StandIn& one : g_standins)
            {
                if (one.resource.get() != src) continue;
                one.last_seen = now;
                g_standin_cycle_known = true;
                g_standin_latest = src;
                return;
            }
            // A new source. Only a picture of the window's own size counts.
            D3D12_RESOURCE_DESC a = dst->GetDesc(), b = src->GetDesc();
            if (b.Dimension != D3D12_RESOURCE_DIMENSION_TEXTURE2D || a.Width != b.Width || a.Height != b.Height) return;
            // One that turns up after the round was complete means the window was rebuilt: start again.
            if (g_standin_cycle_known || g_standins.size() >= 8)
            {
                g_standins.clear();
                g_standin_cycle_known = false;
                ++g_standin_restarts;
            }
            StandIn one;
            one.resource.assign(src);
            one.last_seen = now;
            g_standins.push_back(std::move(one));
            g_standin_latest = src;
        }

        void STDMETHODCALLTYPE hooked_copy_resource(ID3D12GraphicsCommandList* self, ID3D12Resource* dst, ID3D12Resource* src)
        {
            note_copy_into_window(dst, src);
            g_original_copy_resource(self, dst, src);
        }

        void STDMETHODCALLTYPE hooked_copy_region(ID3D12GraphicsCommandList* self, const D3D12_TEXTURE_COPY_LOCATION* dst, UINT x, UINT y, UINT z,
                                                  const D3D12_TEXTURE_COPY_LOCATION* src, const D3D12_BOX* box)
        {
            if (dst && src && x == 0 && y == 0 && dst->Type == D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX && src->Type == D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX)
            {
                note_copy_into_window(dst->pResource, src->pResource);
            }
            g_original_copy_region(self, dst, x, y, z, src, box);
        }

        // The stand-in the game draws on next, if the round is known; also forgets stand-ins that
        // are no longer used (the window was rebuilt), so that they can be freed.
        Com<ID3D12Resource> next_standin()
        {
            Com<ID3D12Resource> next;
            std::lock_guard<std::mutex> lock(g_standin_mutex);
            uint64_t now = g_presents_done.load();
            for (size_t i = 0; i < g_standins.size();)
            {
                if (g_standins[i].last_seen + 64 < now)
                {
                    g_standins.erase(g_standins.begin() + (std::ptrdiff_t)i);
                    g_standin_cycle_known = false;
                }
                else ++i;
            }
            ID3D12Resource* latest = g_standin_latest;
            g_standin_latest = nullptr; // one wipe per copy
            if (!latest || !g_standin_cycle_known || g_standins.size() < 2) return next;
            for (size_t i = 0; i < g_standins.size(); ++i)
            {
                if (g_standins[i].resource.get() != latest) continue;
                next.assign(g_standins[(i + 1) % g_standins.size()].resource.get());
                break;
            }
            return next;
        }

        template <typename Fn>
        bool patch_vtable_slot(void** vtable, int index, void* replacement, Fn& original);

        bool init_graphics(IDXGISwapChain* swapchain)
        {
            Com<ID3D12Device> device;
            if (FAILED(swapchain->GetDevice(IID_PPV_ARGS(device.put()))) || !device)
            {
                HSVR_LOG_EVERY(10.0, "Present on a swap chain that is not Direct3D 12; ignoring it");
                return false;
            }
            if (!g_original_execute)
            {
                HSVR_LOG_EVERY(10.0, "D3D: cannot watch the game's command queues, so VR cannot start");
                return false;
            }
            Com<ID3D12CommandQueue> queue = find_game_queue(swapchain, device.get());
            if (!queue)
            {
                HSVR_LOG_EVERY(10.0, "D3D: waiting to see the game's graphics queue");
                return false;
            }
            g.device = device;
            g.queue = queue;
            g.game_swapchain = swapchain;
            DXGI_SWAP_CHAIN_DESC sd{};
            swapchain->GetDesc(&sd);
            HSVR_LOG("D3D: game swap chain %ux%u format %d, buffers %u, window %p",
                     sd.BufferDesc.Width, sd.BufferDesc.Height, (int)sd.BufferDesc.Format, sd.BufferCount, (void*)sd.OutputWindow);

            XrGraphicsRequirementsD3D12KHR reqs{XR_TYPE_GRAPHICS_REQUIREMENTS_D3D12_KHR};
            XrResult r = xr.GetD3D12GraphicsRequirementsKHR(g_instance, g_system, &reqs);
            if (XR_FAILED(r))
            {
                HSVR_LOG("OpenXR: xrGetD3D12GraphicsRequirementsKHR failed: %s", xr_result_name(r));
                return false;
            }
            LUID device_luid = g.device->GetAdapterLuid();
            if (memcmp(&device_luid, &reqs.adapterLuid, sizeof(LUID)) != 0)
            {
                HSVR_LOG("OpenXR: WARNING the game renders on a different GPU than the headset is attached to");
            }

            XrGraphicsBindingD3D12KHR binding{XR_TYPE_GRAPHICS_BINDING_D3D12_KHR};
            binding.device = g.device.get();
            binding.queue = g.queue.get();
            XrSessionCreateInfo sci{XR_TYPE_SESSION_CREATE_INFO};
            sci.next = &binding;
            sci.systemId = g_system;
            XrSession session = XR_NULL_HANDLE;
            r = xr.CreateSession(g_instance, &sci, &session);
            if (XR_FAILED(r))
            {
                HSVR_LOG("OpenXR: xrCreateSession failed: %s", xr_result_name(r));
                return false;
            }
            g_session.store(session);

            XrReferenceSpaceCreateInfo rsci{XR_TYPE_REFERENCE_SPACE_CREATE_INFO};
            rsci.poseInReferenceSpace.orientation.w = 1.0f;
            rsci.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_LOCAL;
            XrSpace local = XR_NULL_HANDLE;
            r = xr.CreateReferenceSpace(session, &rsci, &local);
            if (XR_FAILED(r))
            {
                HSVR_LOG("OpenXR: xrCreateReferenceSpace failed: %s", xr_result_name(r));
                return false;
            }
            g_local_space.store(local);
            create_hand_actions(session);

            uint32_t format_count = 0;
            xr.EnumerateSwapchainFormats(session, 0, &format_count, nullptr);
            std::vector<int64_t> formats(format_count);
            xr.EnumerateSwapchainFormats(session, format_count, &format_count, formats.data());
            for (DXGI_FORMAT wanted : {DXGI_FORMAT_R8G8B8A8_UNORM_SRGB, DXGI_FORMAT_B8G8R8A8_UNORM_SRGB, DXGI_FORMAT_R8G8B8A8_UNORM, DXGI_FORMAT_B8G8R8A8_UNORM})
            {
                if (std::find(formats.begin(), formats.end(), (int64_t)wanted) != formats.end())
                {
                    g.xr_format = wanted;
                    break;
                }
            }
            if (g.xr_format == DXGI_FORMAT_UNKNOWN)
            {
                HSVR_LOG("OpenXR: no usable 8-bit swapchain format");
                return false;
            }
            HSVR_LOG("OpenXR: session created, swapchain format %d", (int)g.xr_format);

            g.rtv_size = g.device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_RTV);
            g.srv_size = g.device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);
            for (auto& fr : g.ring)
            {
                if (FAILED(g.device->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(fr.allocator.put())))) return false;
            }
            if (FAILED(g.device->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT, g.ring[0].allocator.get(), nullptr, IID_PPV_ARGS(g.cmd.put())))) return false;
            g.cmd->Close();
            if (config().wipe_standins && !g_original_copy_resource)
            {
                // Watch picture copies, to recognise stand-in window pictures (see note_copy_into_window).
                void** list_vtable = *reinterpret_cast<void***>(g.cmd.get());
                HMODULE module = nullptr;
                wchar_t path[MAX_PATH] = L"?";
                if (GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT, (LPCWSTR)list_vtable[17], &module) && module)
                {
                    GetModuleFileNameW(module, path, MAX_PATH);
                }
                const wchar_t* name = wcsrchr(path, L'\\');
                name = name ? name + 1 : path;
                bool whole = patch_vtable_slot(list_vtable, 17, reinterpret_cast<void*>(&hooked_copy_resource), g_original_copy_resource);
                bool region = patch_vtable_slot(list_vtable, 16, reinterpret_cast<void*>(&hooked_copy_region), g_original_copy_region);
                g_standin_watch.store(whole || region);
                HSVR_LOG("Hook: picture copies %s (command list functions are in %ls)", whole && region ? "watched" : "could not be watched", name);
            }
            if (FAILED(g.device->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(g.fence.put())))) return false;
            g.fence_event = CreateEventW(nullptr, FALSE, FALSE, nullptr);
            D3D12_DESCRIPTOR_HEAP_DESC hd{};
            hd.Type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
            hd.NumDescriptors = kRing * kSrvPerFrame;
            hd.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
            if (FAILED(g.device->CreateDescriptorHeap(&hd, IID_PPV_ARGS(g.srv_heap.put())))) return false;
            if (!create_blit_pipeline()) return false;

            int w = shared().eye_width.load(), h = shared().eye_height.load();
            if (!create_xr_swapchain(g.eye[0], w, h, "left eye")) return false;
            if (!create_xr_swapchain(g.eye[1], w, h, "right eye")) return false;
            if (!create_xr_swapchain(g.ui, 1920, 1080, "menu panel")) return false;
            g.ready = true;
            std::thread(idle_pump_thread).detach();
            return true;
        }

        // ---- keeping the session alive while the headset is not showing the game -------------
        // While the headset is asleep or showing something else, SteamVR may hand out frames very
        // slowly. Waiting for them inside the game's Present would drag the whole game down to that
        // pace, so in that state a helper thread runs the (empty) frame loop instead and the game
        // runs free. The presenting thread takes the loop back once the headset shows the game.
        std::atomic<bool> g_pump_allowed{false};
        std::atomic<bool> g_pump_in_frame{false};

        // First successful view query: remember the pose and let the engine side start.
        void probe_first_view(XrTime time)
        {
            if (shared().xr_ready.load()) return;
            FramePose probe;
            if (!locate_views(time, probe)) return;
            {
                std::lock_guard<std::mutex> lock(g_pipe.mutex);
                g_pipe.last_latched = probe;
            }
            g_lead_ns.store(g_frame_period.load() * 2);
            HSVR_LOG("OpenXR: first view. FOV left eye %.1f/%.1f/%.1f/%.1f deg, right eye %.1f/%.1f/%.1f/%.1f deg (left/right/up/down), period %.2f ms",
                     probe.eye[0].angle_left * kRadToDeg, probe.eye[0].angle_right * kRadToDeg, probe.eye[0].angle_up * kRadToDeg, probe.eye[0].angle_down * kRadToDeg,
                     probe.eye[1].angle_left * kRadToDeg, probe.eye[1].angle_right * kRadToDeg, probe.eye[1].angle_up * kRadToDeg, probe.eye[1].angle_down * kRadToDeg,
                     (double)g_frame_period.load() / 1e6);
            shared().xr_ready.store(true);
        }

        void idle_pump_thread()
        {
            for (;;)
            {
                if (!g_pump_allowed.load())
                {
                    Sleep(5);
                    continue;
                }
                g_pump_in_frame.store(true);
                if (g_pump_allowed.load())
                {
                    XrSession session = g_session.load();
                    XrFrameState fs{XR_TYPE_FRAME_STATE};
                    if (XR_SUCCEEDED(xr.WaitFrame(session, nullptr, &fs)) && XR_SUCCEEDED(xr.BeginFrame(session, nullptr)))
                    {
                        g_frame_time.store(fs.predictedDisplayTime);
                        if (fs.predictedDisplayPeriod > 0) g_frame_period.store(fs.predictedDisplayPeriod);
                        probe_first_view(fs.predictedDisplayTime);
                        XrFrameEndInfo fei{XR_TYPE_FRAME_END_INFO};
                        fei.displayTime = fs.predictedDisplayTime;
                        fei.environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
                        xr.EndFrame(session, &fei);
                        g_pump_in_frame.store(false);
                    }
                    else
                    {
                        g_pump_in_frame.store(false);
                        Sleep(50);
                    }
                }
                else
                {
                    g_pump_in_frame.store(false);
                }
            }
        }

        void stop_idle_pump_and_wait()
        {
            g_pump_allowed.store(false);
            for (int i = 0; i < 200 && g_pump_in_frame.load(); ++i) Sleep(5);
        }

        void poll_events()
        {
            XrEventDataBuffer ev{XR_TYPE_EVENT_DATA_BUFFER};
            while (xr.PollEvent(g_instance, &ev) == XR_SUCCESS)
            {
                if (ev.type == XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED)
                {
                    auto* changed = reinterpret_cast<XrEventDataSessionStateChanged*>(&ev);
                    HSVR_LOG("OpenXR: session state -> %d", (int)changed->state);
                    g.session_state = changed->state;
                    if (changed->state == XR_SESSION_STATE_READY)
                    {
                        XrSessionBeginInfo bi{XR_TYPE_SESSION_BEGIN_INFO};
                        bi.primaryViewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
                        XrResult r = xr.BeginSession(g_session.load(), &bi);
                        if (XR_SUCCEEDED(r))
                        {
                            g.session_running = true;
                            shared().xr_running.store(true);
                            shared().recenter_requested.store(true);
                        }
                        else
                        {
                            HSVR_LOG("OpenXR: xrBeginSession failed: %s", xr_result_name(r));
                        }
                    }
                    else if (changed->state == XR_SESSION_STATE_STOPPING)
                    {
                        stop_idle_pump_and_wait();
                        xr.EndSession(g_session.load());
                        g.session_running = false;
                        g.frame_begun = false;
                        shared().xr_running.store(false);
                    }
                    else if (changed->state == XR_SESSION_STATE_EXITING || changed->state == XR_SESSION_STATE_LOSS_PENDING)
                    {
                        g.session_running = false;
                        g.session_lost = true;
                        g.frame_begun = false;
                        shared().xr_running.store(false);
                    }
                }
                else if (ev.type == XR_TYPE_EVENT_DATA_INTERACTION_PROFILE_CHANGED)
                {
                    g_hands.profile_changed.store(true);
                }
                else if (ev.type == XR_TYPE_EVENT_DATA_INSTANCE_LOSS_PENDING)
                {
                    g.session_running = false;
                    g.session_lost = true;
                    shared().xr_running.store(false);
                }
                ev = {XR_TYPE_EVENT_DATA_BUFFER};
            }
        }

        void begin_commands()
        {
            g.ring_index = (g.ring_index + 1) % kRing;
            FrameResources& fr = g.ring[g.ring_index];
            if (g.fence->GetCompletedValue() < fr.fence_value)
            {
                g.fence->SetEventOnCompletion(fr.fence_value, g.fence_event);
                WaitForSingleObject(g.fence_event, 1000);
            }
            fr.allocator->Reset();
            g.cmd->Reset(fr.allocator.get(), nullptr);
        }

        void end_commands()
        {
            g.cmd->Close();
            ID3D12CommandList* lists[] = {g.cmd.get()};
            g.queue->ExecuteCommandLists(1, lists);
            g.queue->Signal(g.fence.get(), ++g.fence_counter);
            g.ring[g.ring_index].fence_value = g.fence_counter;
        }

        void transition(ID3D12Resource* resource, D3D12_RESOURCE_STATES before, D3D12_RESOURCE_STATES after)
        {
            if (before == after) return;
            D3D12_RESOURCE_BARRIER b{};
            b.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
            b.Transition.pResource = resource;
            b.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
            b.Transition.StateBefore = before;
            b.Transition.StateAfter = after;
            g.cmd->ResourceBarrier(1, &b);
        }

        D3D12_GPU_DESCRIPTOR_HANDLE write_srv(int slot, ID3D12Resource* resource, DXGI_FORMAT format)
        {
            int index = g.ring_index * kSrvPerFrame + slot;
            D3D12_CPU_DESCRIPTOR_HANDLE cpu = g.srv_heap->GetCPUDescriptorHandleForHeapStart();
            cpu.ptr += (SIZE_T)index * g.srv_size;
            D3D12_SHADER_RESOURCE_VIEW_DESC desc{};
            desc.Format = format;
            desc.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
            desc.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
            desc.Texture2D.MipLevels = 1;
            g.device->CreateShaderResourceView(resource, &desc, cpu);
            D3D12_GPU_DESCRIPTOR_HANDLE gpu = g.srv_heap->GetGPUDescriptorHandleForHeapStart();
            gpu.ptr += (UINT64)index * g.srv_size;
            return gpu;
        }

        // Acquires the next image of an OpenXR swapchain and draws `source` into it.
        bool blit_to_swapchain(XrSwap& swap, D3D12_GPU_DESCRIPTOR_HANDLE source, const BlitConstants& constants)
        {
            uint32_t index = 0;
            XrSwapchainImageAcquireInfo ai{XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO};
            XrResult r = xr.AcquireSwapchainImage(swap.handle, &ai, &index);
            if (XR_FAILED(r))
            {
                HSVR_LOG_EVERY(5.0, "xrAcquireSwapchainImage failed: %s", xr_result_name(r));
                return false;
            }
            XrSwapchainImageWaitInfo wi{XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO};
            wi.timeout = 100000000; // 100 ms
            r = xr.WaitSwapchainImage(swap.handle, &wi);
            if (r != XR_SUCCESS)
            {
                HSVR_LOG_EVERY(5.0, "xrWaitSwapchainImage: %s", xr_result_name(r));
                XrSwapchainImageReleaseInfo ri{XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
                xr.ReleaseSwapchainImage(swap.handle, &ri);
                return false;
            }
            D3D12_CPU_DESCRIPTOR_HANDLE rtv = swap.rtv_heap->GetCPUDescriptorHandleForHeapStart();
            rtv.ptr += (SIZE_T)index * g.rtv_size;
            D3D12_VIEWPORT vp{0.0f, 0.0f, (float)swap.width, (float)swap.height, 0.0f, 1.0f};
            D3D12_RECT scissor{0, 0, swap.width, swap.height};
            g.cmd->OMSetRenderTargets(1, &rtv, FALSE, nullptr);
            g.cmd->RSSetViewports(1, &vp);
            g.cmd->RSSetScissorRects(1, &scissor);
            g.cmd->SetGraphicsRootDescriptorTable(0, source);
            g.cmd->SetGraphicsRoot32BitConstants(1, sizeof(BlitConstants) / 4, &constants, 0);
            g.cmd->DrawInstanced(3, 1, 0, 0);
            return true;
        }

        void release_swapchain(XrSwap& swap)
        {
            XrSwapchainImageReleaseInfo ri{XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
            xr.ReleaseSwapchainImage(swap.handle, &ri);
        }

        bool current_back_buffer(IDXGISwapChain* swapchain, Com<ID3D12Resource>& back)
        {
            Com<IDXGISwapChain3> sc3;
            if (FAILED(swapchain->QueryInterface(IID_PPV_ARGS(sc3.put())))) return false;
            return SUCCEEDED(sc3->GetBuffer(sc3->GetCurrentBackBufferIndex(), IID_PPV_ARGS(back.put()))) && back;
        }

        // Makes sure we have a private texture matching the back buffer and copies the back buffer into it.
        bool copy_back_buffer(IDXGISwapChain* swapchain)
        {
            Com<ID3D12Resource> back;
            if (!current_back_buffer(swapchain, back)) return false;
            D3D12_RESOURCE_DESC desc = back->GetDesc();
            if (!g.ui_copy || g.ui_copy_width != desc.Width || g.ui_copy_height != desc.Height || g.ui_copy_format != desc.Format)
            {
                D3D12_HEAP_PROPERTIES heap{};
                heap.Type = D3D12_HEAP_TYPE_DEFAULT;
                D3D12_RESOURCE_DESC copy_desc = desc;
                copy_desc.Flags = D3D12_RESOURCE_FLAG_NONE;
                copy_desc.MipLevels = 1;
                copy_desc.Alignment = 0;
                g.ui_copy.reset();
                if (FAILED(g.device->CreateCommittedResource(&heap, D3D12_HEAP_FLAG_NONE, &copy_desc, D3D12_RESOURCE_STATE_COPY_DEST, nullptr, IID_PPV_ARGS(g.ui_copy.put()))))
                {
                    HSVR_LOG_EVERY(5.0, "D3D: could not create the back buffer copy texture");
                    return false;
                }
                g.ui_copy_width = (UINT)desc.Width;
                g.ui_copy_height = desc.Height;
                g.ui_copy_format = desc.Format;
                g.ui_copy_state = D3D12_RESOURCE_STATE_COPY_DEST;
                HSVR_LOG("D3D: back buffer copy %ux%u format %d", g.ui_copy_width, g.ui_copy_height, (int)desc.Format);
            }
            transition(g.ui_copy.get(), g.ui_copy_state, D3D12_RESOURCE_STATE_COPY_DEST);
            transition(back.get(), D3D12_RESOURCE_STATE_PRESENT, D3D12_RESOURCE_STATE_COPY_SOURCE);
            g.cmd->CopyResource(g.ui_copy.get(), back.get());
            transition(back.get(), D3D12_RESOURCE_STATE_COPY_SOURCE, D3D12_RESOURCE_STATE_PRESENT);
            transition(g.ui_copy.get(), D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
            g.ui_copy_state = D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE;
            return true;
        }

        D3D12_CPU_DESCRIPTOR_HANDLE window_rtv(int slot, ID3D12Resource* back)
        {
            D3D12_CPU_DESCRIPTOR_HANDLE rtv{};
            if (!g.window_rtv_heap)
            {
                D3D12_DESCRIPTOR_HEAP_DESC hd{};
                hd.Type = D3D12_DESCRIPTOR_HEAP_TYPE_RTV;
                hd.NumDescriptors = 3;
                if (FAILED(g.device->CreateDescriptorHeap(&hd, IID_PPV_ARGS(g.window_rtv_heap.put())))) return rtv;
            }
            rtv = g.window_rtv_heap->GetCPUDescriptorHandleForHeapStart();
            rtv.ptr += (SIZE_T)slot * g.rtv_size;
            g.device->CreateRenderTargetView(back, nullptr, rtv);
            return rtv;
        }

        // While the eyes have their own target, nothing but the flat menus and HUD is drawn into
        // the game window. This puts the left eye's picture behind them so the monitor still shows
        // the game. `scene_srv`/`ui_srv` are the already prepared source textures (ptr 0 = none).
        void draw_monitor_picture(IDXGISwapChain* swapchain, const PresentItem* item, ID3D12Resource* scene,
                                  D3D12_GPU_DESCRIPTOR_HANDLE scene_srv, D3D12_GPU_DESCRIPTOR_HANDLE ui_srv)
        {
            if (g.window_pipeline_failed) return;
            Com<ID3D12Resource> back;
            if (!current_back_buffer(swapchain, back)) return;
            D3D12_RESOURCE_DESC bd = back->GetDesc();
            if (!g.window_pipeline || g.window_format != bd.Format)
            {
                g.window_pipeline = make_pipeline(bd.Format, false);
                g.window_blend_pipeline = make_pipeline(bd.Format, true);
                g.window_format = bd.Format;
                if (!g.window_pipeline || !g.window_blend_pipeline)
                {
                    g.window_pipeline_failed = true;
                    return;
                }
            }
            D3D12_CPU_DESCRIPTOR_HANDLE rtv = window_rtv(0, back.get());
            if (!rtv.ptr) return;
            transition(back.get(), D3D12_RESOURCE_STATE_PRESENT, D3D12_RESOURCE_STATE_RENDER_TARGET);
            g.cmd->OMSetRenderTargets(1, &rtv, FALSE, nullptr);
            const float black[4] = {0.0f, 0.0f, 0.0f, 1.0f};
            g.cmd->ClearRenderTargetView(rtv, black, 0, nullptr);
            D3D12_RECT scissor{0, 0, (LONG)bd.Width, (LONG)bd.Height};
            g.cmd->RSSetScissorRects(1, &scissor);
            if (scene && scene_srv.ptr && item)
            {
                D3D12_RESOURCE_DESC sd = scene->GetDesc();
                double x0 = 0, y0 = 0, x1 = (double)sd.Width / 2.0, y1 = (double)sd.Height;
                if (item->rect_valid[0])
                {
                    x0 = item->rect[0][0];
                    y0 = item->rect[0][1];
                    x1 = item->rect[0][2];
                    y1 = item->rect[0][3];
                }
                // Fit the eye picture inside the window, keeping its shape.
                double eye_aspect = (x1 - x0) / std::max(1.0, y1 - y0);
                double out_h = (double)bd.Height, out_w = out_h * eye_aspect;
                if (out_w > (double)bd.Width)
                {
                    out_w = (double)bd.Width;
                    out_h = out_w / eye_aspect;
                }
                D3D12_VIEWPORT vp{(float)(((double)bd.Width - out_w) * 0.5), (float)(((double)bd.Height - out_h) * 0.5), (float)out_w, (float)out_h, 0.0f, 1.0f};
                BlitConstants c{};
                c.uv_rect[0] = (float)(x0 / (double)sd.Width);
                c.uv_rect[1] = (float)(y0 / (double)sd.Height);
                c.uv_rect[2] = (float)((x1 - x0) / (double)sd.Width);
                c.uv_rect[3] = (float)((y1 - y0) / (double)sd.Height);
                c.cursor[0] = -1.0f;
                g.cmd->SetPipelineState(g.window_pipeline.get());
                g.cmd->RSSetViewports(1, &vp);
                g.cmd->SetGraphicsRootDescriptorTable(0, scene_srv);
                g.cmd->SetGraphicsRoot32BitConstants(1, sizeof(BlitConstants) / 4, &c, 0);
                g.cmd->DrawInstanced(3, 1, 0, 0);
            }
            if (ui_srv.ptr)
            {
                D3D12_VIEWPORT vp{0.0f, 0.0f, (float)bd.Width, (float)bd.Height, 0.0f, 1.0f};
                BlitConstants c{};
                c.uv_rect[2] = 1.0f;
                c.uv_rect[3] = 1.0f;
                c.params[1] = 1.0f;
                c.params[2] = 2.0f;
                c.cursor[0] = -1.0f;
                g.cmd->SetPipelineState(g.window_blend_pipeline.get());
                g.cmd->RSSetViewports(1, &vp);
                g.cmd->SetGraphicsRootDescriptorTable(0, ui_srv);
                g.cmd->SetGraphicsRoot32BitConstants(1, sizeof(BlitConstants) / 4, &c, 0);
                g.cmd->DrawInstanced(3, 1, 0, 0);
            }
            transition(back.get(), D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_PRESENT);
        }

        void adapt_prediction(const FramePose& pose, const Config& cfg)
        {
            int64_t period = g_frame_period.load();
            if (cfg.predict_frames >= 0.0)
            {
                g_lead_ns.store((int64_t)(cfg.predict_frames * (double)period));
                return;
            }
            if (!pose.valid || pose.predicted_time == 0) return;
            int64_t error = g.frame_time - pose.predicted_time; // > 0: we should have predicted further ahead
            int64_t lead = g_lead_ns.load() + error / 16;
            lead = std::clamp<int64_t>(lead, 0, period * 6);
            g_lead_ns.store(lead);
        }

        // What the engine handed over for the frame being presented right now.
        struct FetchedFrame
        {
            bool separate = false;
            bool have_item = false;
            bool fresh = false;
            PresentItem item;
        };

        // Called exactly once for every Present of the game's window, whether or not the headset
        // is taking frames, so that poses stay matched to pictures by simple counting.
        FetchedFrame fetch_frame(const Config& cfg)
        {
            FetchedFrame f;
            FramePose pose;
            bool have_pose = false;
            {
                std::lock_guard<std::mutex> lock(g_pipe.mutex);
                bool stereo = shared().stereo_installed.load();
                f.separate = g_pipe.separate_mode && stereo;
                // In both modes the pose is taken here, at the Present, in the same way.
                if (stereo) have_pose = take_pose_for_present(pose, 3, cfg.pose_max_age);
                if (f.separate)
                {
                    // Picture and rectangles come from the render thread, which is at most one
                    // frame ahead of the presenting thread.
                    while (g_pipe.present_queue.size() > 2)
                    {
                        g_pipe.present_queue.pop_front();
                        ++g_pipe.dropped_present;
                    }
                    if (!g_pipe.present_queue.empty())
                    {
                        f.item = std::move(g_pipe.present_queue.front());
                        g_pipe.present_queue.pop_front();
                        f.have_item = true;
                    }
                }
                else if (stereo)
                {
                    // The engine draws both eyes side by side into the window itself.
                    f.have_item = have_pose && pose.valid;
                }
            }
            if (f.have_item)
            {
                // No new pose with a new picture (a frame without the 3D world, such as a loading
                // screen): the picture keeps the pose it was last shown with.
                if (have_pose) f.item.pose = pose;
                else if (g.have_last_item) f.item.pose = g.last_item.pose;
                f.fresh = have_pose;
                g.last_item = f.item;
                g.have_last_item = true;
            }
            else if (g.have_last_item)
            {
                f.item = g.last_item; // nothing new from the engine: show the last image again
                f.have_item = true;
            }
            return f;
        }

        // ---- pictures for troubleshooting ----------------------------------------------------------
        // Every `capture_seconds` (settings file; 0 = off) the menu layer (exactly what the floating
        // panel is made from) and the finished monitor picture are saved as half-size .bmp files in
        // Mods\HalfSwordVR\captures. The GPU copies them into buffers we can read; a helper thread
        // writes the files, so the game does not wait for the disk.
        constexpr int kCaptureKinds = 4; // 0: menu layer, 1: monitor picture, 2: next window buffer just after it was wiped, 3: menu layer one Present later
        constexpr int kCaptureFiles = 48;
        struct CaptureBuffer
        {
            Com<ID3D12Resource> buffer;
            UINT64 bytes = 0;
            D3D12_PLACED_SUBRESOURCE_FOOTPRINT footprint{};
            bool filled = false;
        };
        struct CaptureState
        {
            CaptureBuffer kind[kCaptureKinds];
            bool recorded = false; // copy commands are in the command list being built
            bool pending = false;  // submitted, waiting for the GPU
            // Menus-only pictures come as a set of three, to show where old content in the window
            // buffer comes from: the menu layer, the next buffer right after the wipe, and the
            // menu layer of the following Present (the same buffer after the game drew into it).
            bool want_cleared = false;
            bool want_next = false;
            void* shot_buffer = nullptr; // window buffer of the Present the set started with
            UINT64 fence_value = 0;
            std::atomic<bool> writing{false};
            double last_time = -1000.0;
            int index = 0;
        };
        CaptureState g_capture;

        bool capture_format_supported(DXGI_FORMAT format)
        {
            switch (format)
            {
            case DXGI_FORMAT_R10G10B10A2_UNORM:
            case DXGI_FORMAT_R8G8B8A8_UNORM:
            case DXGI_FORMAT_R8G8B8A8_UNORM_SRGB:
            case DXGI_FORMAT_B8G8R8A8_UNORM:
            case DXGI_FORMAT_B8G8R8A8_UNORM_SRGB:
                return true;
            default:
                return false;
            }
        }

        // Adds "copy this texture into a readable buffer" to the open command list.
        void record_capture(int which, ID3D12Resource* source, D3D12_RESOURCE_STATES state)
        {
            CaptureBuffer& cb = g_capture.kind[which];
            D3D12_RESOURCE_DESC desc = source->GetDesc();
            if (desc.SampleDesc.Count != 1 || !capture_format_supported(desc.Format))
            {
                HSVR_LOG_EVERY(60.0, "Capture: picture format %d is not supported", (int)desc.Format);
                return;
            }
            D3D12_PLACED_SUBRESOURCE_FOOTPRINT footprint{};
            UINT64 total = 0;
            g.device->GetCopyableFootprints(&desc, 0, 1, 0, &footprint, nullptr, nullptr, &total);
            if (total == 0) return;
            if (!cb.buffer || cb.bytes < total)
            {
                D3D12_HEAP_PROPERTIES heap{};
                heap.Type = D3D12_HEAP_TYPE_READBACK;
                D3D12_RESOURCE_DESC bd{};
                bd.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
                bd.Width = total;
                bd.Height = 1;
                bd.DepthOrArraySize = 1;
                bd.MipLevels = 1;
                bd.SampleDesc.Count = 1;
                bd.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
                cb.buffer.reset();
                cb.bytes = 0;
                if (FAILED(g.device->CreateCommittedResource(&heap, D3D12_HEAP_FLAG_NONE, &bd, D3D12_RESOURCE_STATE_COPY_DEST, nullptr, IID_PPV_ARGS(cb.buffer.put()))))
                {
                    HSVR_LOG_EVERY(60.0, "Capture: could not create a read-back buffer");
                    return;
                }
                cb.bytes = total;
            }
            cb.footprint = footprint;
            D3D12_TEXTURE_COPY_LOCATION dst{};
            dst.pResource = cb.buffer.get();
            dst.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
            dst.PlacedFootprint = footprint;
            D3D12_TEXTURE_COPY_LOCATION src{};
            src.pResource = source;
            src.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
            src.SubresourceIndex = 0;
            transition(source, state, D3D12_RESOURCE_STATE_COPY_SOURCE);
            g.cmd->CopyTextureRegion(&dst, 0, 0, 0, &src, nullptr);
            transition(source, D3D12_RESOURCE_STATE_COPY_SOURCE, state);
            cb.filled = true;
            g_capture.recorded = true;
        }

        bool capture_due(const Config& cfg)
        {
            if (cfg.capture_seconds <= 0.0) return false;
            if (g_capture.pending || g_capture.writing.load()) return false;
            return now_seconds() - g_capture.last_time >= cfg.capture_seconds;
        }

        // Writes one read-back buffer as a 24-bit .bmp at half size. Runs on the helper thread.
        bool write_capture_file(const std::wstring& path, ID3D12Resource* buffer, const D3D12_PLACED_SUBRESOURCE_FOOTPRINT& fp, UINT64 bytes)
        {
            void* mapped = nullptr;
            D3D12_RANGE read{0, (SIZE_T)bytes};
            if (FAILED(buffer->Map(0, &read, &mapped)) || !mapped) return false;
            const uint8_t* base = static_cast<const uint8_t*>(mapped) + fp.Offset;
            const int step = 2;
            const int width = (int)fp.Footprint.Width / step, height = (int)fp.Footprint.Height / step;
            const int row_bytes = (width * 3 + 3) & ~3;
            std::vector<uint8_t> out((size_t)row_bytes * (size_t)std::max(0, height));
            const DXGI_FORMAT format = fp.Footprint.Format;
            for (int y = 0; y < height; ++y)
            {
                const uint8_t* row = base + (size_t)(y * step) * fp.Footprint.RowPitch;
                uint8_t* dst = out.data() + (size_t)(height - 1 - y) * row_bytes; // .bmp rows run bottom to top
                for (int x = 0; x < width; ++x)
                {
                    const uint8_t* px = row + (size_t)(x * step) * 4;
                    uint8_t r, gr, b;
                    if (format == DXGI_FORMAT_R10G10B10A2_UNORM)
                    {
                        uint32_t v;
                        memcpy(&v, px, 4);
                        r = (uint8_t)((v & 0x3FF) >> 2);
                        gr = (uint8_t)(((v >> 10) & 0x3FF) >> 2);
                        b = (uint8_t)(((v >> 20) & 0x3FF) >> 2);
                    }
                    else if (format == DXGI_FORMAT_B8G8R8A8_UNORM || format == DXGI_FORMAT_B8G8R8A8_UNORM_SRGB)
                    {
                        b = px[0];
                        gr = px[1];
                        r = px[2];
                    }
                    else
                    {
                        r = px[0];
                        gr = px[1];
                        b = px[2];
                    }
                    dst[x * 3 + 0] = b;
                    dst[x * 3 + 1] = gr;
                    dst[x * 3 + 2] = r;
                }
            }
            D3D12_RANGE none{0, 0};
            buffer->Unmap(0, &none);
            if (width <= 0 || height <= 0) return false;

            BITMAPFILEHEADER file{};
            BITMAPINFOHEADER info{};
            file.bfType = 0x4D42;
            file.bfOffBits = sizeof(file) + sizeof(info);
            file.bfSize = (DWORD)(file.bfOffBits + out.size());
            info.biSize = sizeof(info);
            info.biWidth = width;
            info.biHeight = height;
            info.biPlanes = 1;
            info.biBitCount = 24;
            info.biSizeImage = (DWORD)out.size();
            FILE* f = _wfopen(path.c_str(), L"wb");
            if (!f) return false;
            bool ok = fwrite(&file, sizeof(file), 1, f) == 1 && fwrite(&info, sizeof(info), 1, f) == 1 && fwrite(out.data(), 1, out.size(), f) == out.size();
            fclose(f);
            return ok;
        }

        // Called once per Present, before new commands are built.
        void service_capture()
        {
            if (!g_capture.pending || g_capture.want_next || g.fence->GetCompletedValue() < g_capture.fence_value) return;
            g_capture.pending = false;
            g_capture.want_cleared = false;
            struct Job
            {
                ID3D12Resource* buffer;
                D3D12_PLACED_SUBRESOURCE_FOOTPRINT footprint;
                UINT64 bytes;
            };
            Job jobs[kCaptureKinds]{};
            for (int i = 0; i < kCaptureKinds; ++i)
            {
                CaptureBuffer& cb = g_capture.kind[i];
                if (cb.filled) jobs[i] = {cb.buffer.get(), cb.footprint, cb.bytes};
                cb.filled = false;
            }
            int index = g_capture.index;
            g_capture.index = (g_capture.index + 1) % kCaptureFiles;
            g_capture.writing.store(true);
            // The buffers are left alone until `writing` is false again (see capture_due).
            std::thread([jobs, index]() {
                std::wstring folder = mod_dir() + L"captures";
                CreateDirectoryW(folder.c_str(), nullptr);
                static const wchar_t* const names[kCaptureKinds] = {L"menu", L"monitor", L"cleared", L"next"};
                int saved = 0;
                for (int i = 0; i < kCaptureKinds; ++i)
                {
                    if (!jobs[i].buffer) continue;
                    wchar_t name[64];
                    swprintf(name, 64, L"\\capture_%02d_%ls.bmp", index, names[i]);
                    if (write_capture_file(folder + name, jobs[i].buffer, jobs[i].footprint, jobs[i].bytes)) ++saved;
                }
                HSVR_LOG("Capture %02d saved (%d picture(s))", index, saved);
                g_capture.writing.store(false);
            }).detach();
        }

        // Runs right after the game's Present while the eyes have their own target: wipes the
        // window buffer the game will draw its menus and HUD into next, so that what we copy from
        // it next frame is only that frame's menus on black.
        void clear_next_window_buffer(IDXGISwapChain* swapchain)
        {
            std::lock_guard<std::mutex> lock(g_present_mutex);
            if (!g.ready || swapchain != g.game_swapchain) return;
            {
                std::lock_guard<std::mutex> pipe_lock(g_pipe.mutex);
                if (!g_pipe.separate_mode || !shared().stereo_installed.load()) return;
            }
            Com<IDXGISwapChain3> sc3;
            if (FAILED(swapchain->QueryInterface(IID_PPV_ARGS(sc3.put())))) return;
            UINT index = sc3->GetCurrentBackBufferIndex();
            Com<ID3D12Resource> back;
            if (FAILED(sc3->GetBuffer(index, IID_PPV_ARGS(back.put()))) || !back) return;
            if (!g.swapchain_logged)
            {
                g.swapchain_logged = true;
                DXGI_SWAP_CHAIN_DESC d{};
                if (SUCCEEDED(swapchain->GetDesc(&d)))
                {
                    HSVR_LOG("Window: swap chain has %u buffers, %ux%u, format %d, swap effect %d, flags 0x%X, windowed %d", d.BufferCount, d.BufferDesc.Width,
                             d.BufferDesc.Height, (int)d.BufferDesc.Format, (int)d.SwapEffect, d.Flags, (int)d.Windowed);
                }
            }
            // Which pictures are the window's own, for recognising copies into them.
            {
                DXGI_SWAP_CHAIN_DESC d{};
                UINT count = SUCCEEDED(swapchain->GetDesc(&d)) ? d.BufferCount : 0;
                for (UINT i = 0; i < 8; ++i)
                {
                    Com<ID3D12Resource> one;
                    if (i < count) sc3->GetBuffer(i, IID_PPV_ARGS(one.put()));
                    g_window_buffers[i].store(one.get(), std::memory_order_relaxed);
                }
            }
            Com<ID3D12Resource> standin = next_standin();
            begin_commands();
            if (standin)
            {
                D3D12_CPU_DESCRIPTOR_HANDLE standin_rtv = window_rtv(2, standin.get());
                if (standin_rtv.ptr)
                {
                    transition(standin.get(), D3D12_RESOURCE_STATE_PRESENT, D3D12_RESOURCE_STATE_RENDER_TARGET);
                    const float clear[4] = {0.0f, 0.0f, 0.0f, 0.0f};
                    g.cmd->ClearRenderTargetView(standin_rtv, clear, 0, nullptr);
                    transition(standin.get(), D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_PRESENT);
                    std::lock_guard<std::mutex> standin_lock(g_standin_mutex);
                    if (g_standin_wipes++ == 0)
                    {
                        HSVR_LOG("Window: the game draws its menus on %zu stand-in pictures that an add-on copies into the window; the next one is now wiped each frame as well",
                                 g_standins.size());
                    }
                }
            }
            D3D12_CPU_DESCRIPTOR_HANDLE rtv = window_rtv(1, back.get());
            bool shot = false;
            if (rtv.ptr)
            {
                transition(back.get(), D3D12_RESOURCE_STATE_PRESENT, D3D12_RESOURCE_STATE_RENDER_TARGET);
                const float clear[4] = {0.0f, 0.0f, 0.0f, 0.0f};
                g.cmd->ClearRenderTargetView(rtv, clear, 0, nullptr);
                transition(back.get(), D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_PRESENT);
                g.wiped_buffer = back.get();
                g.wiped_index = index;
                ++g.wipes;
                if (g_capture.pending && g_capture.want_cleared)
                {
                    g_capture.want_cleared = false;
                    record_capture(2, back.get(), D3D12_RESOURCE_STATE_PRESENT);
                    g_capture.recorded = false;
                    shot = true;
                    HSVR_LOG("Window: picture set %02d: Present drew into buffer %p, wiped next buffer %p (index %u)", g_capture.index, g_capture.shot_buffer,
                             (void*)back.get(), index);
                }
            }
            end_commands();
            if (shot) g_capture.fence_value = g.fence_counter;
        }

        // Finishes the OpenXR frame that is open: copies the engine's eyes (and the flat UI) and submits.
        // With `submit` false nothing goes to the headset (the helper thread owns the frame loop
        // then); only the monitor picture is put together.
        void end_frame(IDXGISwapChain* swapchain, const Config& cfg, FetchedFrame& fetched, bool submit)
        {
            bool separate = fetched.separate;
            service_capture();
            if (!submit && !separate) return;
            const bool render = submit && g.should_render;
            bool have_item = fetched.have_item;
            PresentItem& item = fetched.item;
            if (fetched.fresh) adapt_prediction(item.pose, cfg);

            XrCompositionLayerProjectionView views[2]{{XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW}, {XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW}};
            XrCompositionLayerProjection projection{XR_TYPE_COMPOSITION_LAYER_PROJECTION};
            XrCompositionLayerQuad quad{XR_TYPE_COMPOSITION_LAYER_QUAD};
            const XrCompositionLayerBaseHeader* layers[2]{};
            uint32_t layer_count = 0;
            bool eyes_acquired[2]{false, false};
            bool ui_acquired = false;

            {
                begin_commands();
                ID3D12DescriptorHeap* heaps[] = {g.srv_heap.get()};
                g.cmd->SetDescriptorHeaps(1, heaps);
                g.cmd->SetGraphicsRootSignature(g.root_signature.get());
                g.cmd->SetPipelineState(g.pipeline.get());
                g.cmd->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

                // With the eyes in their own target the game window holds only the flat menus and
                // HUD: copy it for the floating panel, then put the monitor picture together.
                // Is a menu up? Either Windows is showing its mouse pointer inside the game window
                // or the game says it wants one. (The mod draws no pointer of its own: the game's
                // own pointer is visible in the headset.)
                bool pointer_showing = false;
                bool game_pointer = shared().game_wants_pointer.load();
                if (separate)
                {
                    DXGI_SWAP_CHAIN_DESC scd{};
                    swapchain->GetDesc(&scd);
                    CURSORINFO ci{};
                    ci.cbSize = sizeof(ci);
                    RECT client{};
                    if (scd.OutputWindow && GetCursorInfo(&ci) && (ci.flags & CURSOR_SHOWING) && GetClientRect(scd.OutputWindow, &client) &&
                        client.right > 0 && client.bottom > 0)
                    {
                        POINT p = ci.ptScreenPos;
                        ScreenToClient(scd.OutputWindow, &p);
                        pointer_showing = p.x >= 0 && p.y >= 0 && p.x < client.right && p.y < client.bottom;
                    }
                }
                // The panel shows the game window: menus, and during a fight the game's own edge
                // tints (heat, cold, pain), which tell the player things, so it is on by default.
                bool menu_showing = shared().menu_up.load() || pointer_showing || game_pointer;
                bool toggled = shared().ui_toggled.load();
                // The End key hides the panel during play only: a menu always brings it up, or a
                // player who hid it would be left with no way to see the pause menu.
                bool panel_on = cfg.ui_panel_auto ? (menu_showing || toggled) : (!toggled || menu_showing);
                bool want_ui = separate && cfg.ui_panel && panel_on;
                bool want_monitor = separate && cfg.mirror_window;
                bool need_back_buffer = separate ? (want_ui || want_monitor) : (render && have_item);
                bool have_back_buffer = need_back_buffer && copy_back_buffer(swapchain);

                ID3D12Resource* scene = separate ? item.scene.get() : (have_back_buffer ? g.ui_copy.get() : nullptr);
                if (!have_item) scene = nullptr;
                D3D12_GPU_DESCRIPTOR_HANDLE scene_srv{}, ui_srv{};
                bool scene_is_float = false, ui_is_float = false;
                if (scene)
                {
                    D3D12_RESOURCE_DESC sd = scene->GetDesc();
                    DXGI_FORMAT srv_format = sampling_format(sd.Format, scene_is_float);
                    if (scene != g.logged_scene)
                    {
                        g.logged_scene = scene;
                        HSVR_LOG("D3D: engine eye texture %p %llux%u format %d flags 0x%X (sampling as %d)",
                                 (void*)scene, (unsigned long long)sd.Width, sd.Height, (int)sd.Format, (unsigned)sd.Flags, (int)srv_format);
                    }
                    scene_srv = write_srv(0, scene, srv_format);
                }
                if (separate && have_back_buffer)
                {
                    ui_srv = write_srv(1, g.ui_copy.get(), sampling_format(g.ui_copy_format, ui_is_float));
                }

                // ---- eyes ----
                if (render && have_item && item.pose.valid && scene)
                {
                    D3D12_RESOURCE_DESC sd = scene->GetDesc();
                    D3D12_GPU_DESCRIPTOR_HANDLE srv = scene_srv;
                    bool is_float = scene_is_float;
                    bool decode = cfg.gamma_mode == 1 || (cfg.gamma_mode == 0 && !is_float);
                    if (!is_srgb_format(g.xr_format)) decode = false;
                    for (int e = 0; e < 2; ++e)
                    {
                        double x0, y0, x1, y1;
                        if (item.rect_valid[e])
                        {
                            x0 = item.rect[e][0];
                            y0 = item.rect[e][1];
                            x1 = item.rect[e][2];
                            y1 = item.rect[e][3];
                        }
                        else
                        {
                            double half = (double)sd.Width / 2.0;
                            x0 = half * e;
                            y0 = 0;
                            x1 = half * (e + 1);
                            y1 = (double)sd.Height;
                        }
                        BlitConstants c{};
                        c.uv_rect[0] = (float)(x0 / (double)sd.Width);
                        c.uv_rect[1] = (float)(y0 / (double)sd.Height);
                        c.uv_rect[2] = (float)((x1 - x0) / (double)sd.Width);
                        c.uv_rect[3] = (float)((y1 - y0) / (double)sd.Height);
                        c.params[0] = decode ? 1.0f : 0.0f;
                        c.cursor[0] = -1.0f;
                        eyes_acquired[e] = blit_to_swapchain(g.eye[e], srv, c);

                        const XrEyeRaw& eye = item.pose.eye[e];
                        views[e].pose.position = {eye.px, eye.py, eye.pz};
                        views[e].pose.orientation = {eye.qx, eye.qy, eye.qz, eye.qw};
                        views[e].fov = {eye.angle_left, eye.angle_right, eye.angle_up, eye.angle_down};
                        views[e].subImage.swapchain = g.eye[e].handle;
                        views[e].subImage.imageRect = {{0, 0}, {g.eye[e].width, g.eye[e].height}};
                    }
                    if (eyes_acquired[0] && eyes_acquired[1])
                    {
                        projection.space = g_local_space.load();
                        projection.viewCount = 2;
                        projection.views = views;
                        layers[layer_count++] = reinterpret_cast<const XrCompositionLayerBaseHeader*>(&projection);
                    }
                }
                else if (render)
                {
                    shared().frames_without_image.fetch_add(1);
                }

                // ---- flat UI panel ----
                if (render && want_ui && ui_srv.ptr)
                {
                    bool is_float = ui_is_float;
                    D3D12_GPU_DESCRIPTOR_HANDLE srv = ui_srv;
                    BlitConstants c{};
                    c.uv_rect[2] = 1.0f;
                    c.uv_rect[3] = 1.0f;
                    c.params[0] = (is_srgb_format(g.xr_format) && !is_float) ? 1.0f : 0.0f;
                    c.params[1] = cfg.ui_use_alpha ? 2.0f : 1.0f;
                    c.params[2] = (float)cfg.ui_opacity_gain;
                    c.cursor[0] = -1.0f;
                    // A menu (the game is paused or wants a mouse pointer): put a dark sheet behind
                    // the panel so every button can be read whatever is behind it.
                    if (pointer_showing || game_pointer || shared().game_paused.load()) c.params[3] = (float)cfg.ui_menu_backdrop;
                    ui_acquired = blit_to_swapchain(g.ui, srv, c);
                    if (ui_acquired)
                    {
                        float pos0[3];
                        float yaw;
                        {
                            std::lock_guard<std::mutex> lock(g_recenter_mutex);
                            memcpy(pos0, g_recenter_xr_pos, sizeof(pos0));
                            yaw = g_recenter_xr_yaw;
                        }
                        float d = (float)cfg.ui_distance_m;
                        quad.layerFlags = XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT; // premultiplied
                        quad.space = g_local_space.load();
                        quad.eyeVisibility = XR_EYE_VISIBILITY_BOTH;
                        quad.subImage.swapchain = g.ui.handle;
                        quad.subImage.imageRect = {{0, 0}, {g.ui.width, g.ui.height}};
                        quad.pose.orientation = {0.0f, std::sin(yaw * 0.5f), 0.0f, std::cos(yaw * 0.5f)};
                        quad.pose.position = {pos0[0] - d * std::sin(yaw), pos0[1] + (float)cfg.ui_height_offset_m, pos0[2] - d * std::cos(yaw)};
                        float aspect = (float)g.ui_copy_height / (float)std::max<UINT>(1, g.ui_copy_width);
                        quad.size = {(float)cfg.ui_width_m, (float)cfg.ui_width_m * aspect};
                        layers[layer_count++] = reinterpret_cast<const XrCompositionLayerBaseHeader*>(&quad);
                    }
                }

                if (want_monitor && have_back_buffer) draw_monitor_picture(swapchain, have_item ? &item : nullptr, scene, scene_srv, ui_srv);

                if (separate && have_back_buffer)
                {
                    // Is the buffer the game drew into the one that was wiped after the last Present?
                    Com<ID3D12Resource> now_back;
                    if (g.wiped_buffer && current_back_buffer(swapchain, now_back))
                    {
                        ++g.wipe_checks;
                        if (now_back.get() != g.wiped_buffer) ++g.wipe_mismatches;
                    }
                    if (g_capture.pending && g_capture.want_next)
                    {
                        // Third picture of a set: the buffer that was wiped, after the game drew its menus into it.
                        g_capture.want_next = false;
                        record_capture(3, g.ui_copy.get(), g.ui_copy_state);
                        HSVR_LOG("Window: picture set %02d: the following Present drew into buffer %p (wiped was %p)", g_capture.index, (void*)now_back.get(),
                                 (void*)g.wiped_buffer);
                    }
                }
                else
                {
                    g_capture.want_next = false;
                }
                if (separate && have_back_buffer && capture_due(cfg) && (!cfg.capture_menus_only || game_pointer))
                {
                    g_capture.last_time = now_seconds();
                    record_capture(0, g.ui_copy.get(), g.ui_copy_state);
                    Com<ID3D12Resource> back;
                    bool have_back = current_back_buffer(swapchain, back);
                    if (!cfg.capture_menus_only && want_monitor && have_back)
                    {
                        record_capture(1, back.get(), D3D12_RESOURCE_STATE_PRESENT);
                    }
                    if (cfg.capture_menus_only)
                    {
                        g_capture.want_cleared = true;
                        g_capture.want_next = true;
                        g_capture.shot_buffer = have_back ? (void*)back.get() : nullptr;
                    }
                }

                end_commands();
                if (g_capture.recorded)
                {
                    g_capture.recorded = false;
                    g_capture.pending = true;
                    g_capture.fence_value = g.fence_counter;
                }
                for (int e = 0; e < 2; ++e)
                {
                    if (eyes_acquired[e]) release_swapchain(g.eye[e]);
                }
                if (ui_acquired) release_swapchain(g.ui);
            }

            if (!submit) return;
            XrFrameEndInfo fei{XR_TYPE_FRAME_END_INFO};
            fei.displayTime = g.frame_time;
            fei.environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
            fei.layerCount = layer_count;
            fei.layers = layer_count ? layers : nullptr;
            XrResult r = xr.EndFrame(g_session.load(), &fei);
            if (XR_FAILED(r)) HSVR_LOG_EVERY(5.0, "xrEndFrame failed: %s", xr_result_name(r));
            g.frame_begun = false;
            if (layer_count) shared().frames_submitted.fetch_add(1);
        }

        void begin_frame()
        {
            XrSession session = g_session.load();
            XrFrameState fs{XR_TYPE_FRAME_STATE};
            XrResult r = xr.WaitFrame(session, nullptr, &fs);
            if (XR_FAILED(r))
            {
                HSVR_LOG_EVERY(5.0, "xrWaitFrame failed: %s", xr_result_name(r));
                return;
            }
            r = xr.BeginFrame(session, nullptr);
            if (XR_FAILED(r))
            {
                HSVR_LOG_EVERY(5.0, "xrBeginFrame failed: %s", xr_result_name(r));
                return;
            }
            g.frame_begun = true;
            g.should_render = fs.shouldRender == XR_TRUE;
            g.frame_time = fs.predictedDisplayTime;
            g_frame_time.store(fs.predictedDisplayTime);
            if (fs.predictedDisplayPeriod > 0) g_frame_period.store(fs.predictedDisplayPeriod);

            probe_first_view(fs.predictedDisplayTime);
        }

        void log_statistics()
        {
            uint64_t dropped_game, dropped_present, reused, stale, age[5];
            size_t game_depth, present_depth, depth_min, depth_max;
            {
                std::lock_guard<std::mutex> lock(g_pipe.mutex);
                dropped_game = g_pipe.dropped_game;
                dropped_present = g_pipe.dropped_present;
                reused = g_pipe.reused_render;
                stale = g_pipe.dropped_stale;
                game_depth = g_pipe.game_queue.size();
                present_depth = g_pipe.present_queue.size();
                depth_min = g_pipe.depth_min;
                depth_max = g_pipe.depth_max;
                for (int i = 0; i < 5; ++i)
                {
                    age[i] = g_pipe.age_count[i];
                    g_pipe.age_count[i] = 0;
                }
                g_pipe.depth_min = 99;
                g_pipe.depth_max = 0;
            }
            if (depth_min > depth_max) depth_min = depth_max;
            HSVR_LOG("Stats: state %d, presents %llu, submitted %llu, without image %llu, pose queue %zu/%zu, dropped %llu/%llu, reused %llu, lead %.1f ms",
                     (int)g.session_state, (unsigned long long)g.presents, (unsigned long long)shared().frames_submitted.load(),
                     (unsigned long long)shared().frames_without_image.load(), game_depth, present_depth,
                     (unsigned long long)dropped_game, (unsigned long long)dropped_present, (unsigned long long)reused,
                     (double)g_lead_ns.load() / 1e6);
            HSVR_LOG("Window: wiped %llu, checked %llu, the game drew into a different buffer than the wiped one %llu time(s)", (unsigned long long)g.wipes,
                     (unsigned long long)g.wipe_checks, (unsigned long long)g.wipe_mismatches);
            {
                std::lock_guard<std::mutex> standin_lock(g_standin_mutex);
                HSVR_LOG("Window: stand-in pictures %zu (order %s), copies into the window seen %llu, stand-ins wiped %llu, started over %llu time(s)", g_standins.size(),
                         g_standin_cycle_known ? "known" : "not known yet", (unsigned long long)g_standin_copies, (unsigned long long)g_standin_wipes,
                         (unsigned long long)g_standin_restarts);
            }
            HSVR_LOG("Pairing: poses waiting at a Present %zu to %zu, too old and dropped %llu (all session), Presents between pose and picture 0:%llu 1:%llu 2:%llu 3:%llu 4+:%llu",
                     depth_min, depth_max, (unsigned long long)stale, (unsigned long long)age[0], (unsigned long long)age[1],
                     (unsigned long long)age[2], (unsigned long long)age[3], (unsigned long long)age[4]);
        }

        // Runs inside the game's Present call, on whichever thread presents. Returns true while VR is driving frame timing.
        bool on_present(IDXGISwapChain* swapchain)
        {
            if (!g_vr_available || g_vr_failed) return false;
            std::lock_guard<std::mutex> lock(g_present_mutex);
            if (!g.ready)
            {
                if (g.session_lost) return false;
                if (g.game_swapchain) return false; // init was tried and failed part-way
                if (!init_graphics(swapchain))
                {
                    if (g.game_swapchain)
                    {
                        HSVR_LOG("VR start-up failed; the game continues flat");
                        g_vr_failed = true;
                    }
                    return false;
                }
            }
            if (swapchain != g.game_swapchain) return false;
            ++g.presents;
            t_game_present = true;
            Config cfg = config();
            FetchedFrame fetched = fetch_frame(cfg);
            if (!g.session_lost) poll_events();
            if (log_throttle("stats", 10.0)) log_statistics();
            bool showing = g.session_running && !g.session_lost &&
                           (!cfg.idle_helper || g.session_state == XR_SESSION_STATE_VISIBLE || g.session_state == XR_SESSION_STATE_FOCUSED);
            if (!showing)
            {
                // The headset is not showing the game: finish any frame we had open, then let
                // the helper thread keep the session alive while the game runs at its own pace.
                end_frame(swapchain, cfg, fetched, g.frame_begun);
                g_pump_allowed.store(g.session_running && !g.session_lost);
                return false;
            }
            g_pump_allowed.store(false);
            if (g_pump_in_frame.load())
            {
                // The helper thread is finishing its last frame; take over on the next Present.
                end_frame(swapchain, cfg, fetched, false);
                return false;
            }
            end_frame(swapchain, cfg, fetched, g.frame_begun);
            begin_frame();
            return true;
        }

        // While the headset is not pacing the game (asleep, or SteamVR gone) and the engine is
        // drawing two eyes with its own frame cap off, hold the game to about the headset's rate.
        double g_next_unpaced_present = 0.0;
        void limit_unpaced_frame_rate(IDXGISwapChain* swapchain)
        {
            if (swapchain != g.game_swapchain || !shared().stereo_installed.load()) return;
            double period = (double)g_frame_period.load() / 1e9;
            if (period < 0.004 || period > 0.05) period = 1.0 / 72.0;
            double now = now_seconds();
            if (g_next_unpaced_present < now - period || g_next_unpaced_present > now + period) g_next_unpaced_present = now;
            double wait = g_next_unpaced_present - now;
            if (wait > 0.0015) Sleep((DWORD)(wait * 1000.0));
            g_next_unpaced_present += period;
        }

        // Who presents the game window: the game itself, or an add-on that sits between the game
        // and the window (the frame generation parts of FSR and DLSS replace the window's
        // pictures with their own and copy them over at the Present). Written when it changes.
        void log_present_caller(void* return_address, IDXGISwapChain* swapchain)
        {
            static HMODULE last_module = nullptr;
            static int lines_left = 6;
            HMODULE module = nullptr;
            GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT, (LPCWSTR)return_address, &module);
            if (lines_left <= 0 || (module == last_module && lines_left < 6)) return;
            last_module = module;
            --lines_left;
            wchar_t path[MAX_PATH] = L"?";
            if (module) GetModuleFileNameW(module, path, MAX_PATH);
            const wchar_t* name = wcsrchr(path, L'\\');
            name = name ? name + 1 : path;
            static const wchar_t* const add_ons[] = {L"amd_fidelityfx_framegeneration_dx12.dll", L"amd_fidelityfx_dx12.dll", L"sl.interposer.dll", L"sl.dlss_g.dll", L"sl.common.dll",
                                                     L"nvngx_dlssg.dll"};
            std::string loaded;
            for (const wchar_t* add_on : add_ons)
            {
                if (!GetModuleHandleW(add_on)) continue;
                if (!loaded.empty()) loaded += ", ";
                for (const wchar_t* c = add_on; *c; ++c) loaded += (char)*c;
            }
            HSVR_LOG("Window: the Present of swap chain %p is called from %ls; frame generation add-ons loaded: %s", (void*)swapchain, name,
                     loaded.empty() ? "none" : loaded.c_str());
        }

        HRESULT STDMETHODCALLTYPE hooked_present(IDXGISwapChain* self, UINT sync_interval, UINT flags)
        {
            log_present_caller(HSVR_RETURN_ADDRESS(), self);
            if (t_in_present || (flags & DXGI_PRESENT_TEST)) return g_original_present(self, sync_interval, flags);
            t_in_present = true;
            bool vr_paced = on_present(self);
            if (!vr_paced) limit_unpaced_frame_rate(self);
            HRESULT hr = g_original_present(self, vr_paced ? 0 : sync_interval, flags);
            if (SUCCEEDED(hr)) clear_next_window_buffer(self);
            if (t_game_present)
            {
                t_game_present = false;
                g_presents_done.fetch_add(1);
            }
            t_in_present = false;
            return hr;
        }

        HRESULT STDMETHODCALLTYPE hooked_present1(IDXGISwapChain1* self, UINT sync_interval, UINT flags, const DXGI_PRESENT_PARAMETERS* params)
        {
            log_present_caller(HSVR_RETURN_ADDRESS(), self);
            if (t_in_present || (flags & DXGI_PRESENT_TEST)) return g_original_present1(self, sync_interval, flags, params);
            t_in_present = true;
            bool vr_paced = on_present(self);
            if (!vr_paced) limit_unpaced_frame_rate(self);
            HRESULT hr = g_original_present1(self, vr_paced ? 0 : sync_interval, flags, params);
            if (SUCCEEDED(hr)) clear_next_window_buffer(self);
            if (t_game_present)
            {
                t_game_present = false;
                g_presents_done.fetch_add(1);
            }
            t_in_present = false;
            return hr;
        }

        // Swaps one entry of a function table. The original is stored first, because the game may
        // call through the table on another thread the moment the new entry is written.
        template <typename Fn>
        bool patch_vtable_slot(void** vtable, int index, void* replacement, Fn& original)
        {
            DWORD old_protect = 0;
            if (!VirtualProtect(&vtable[index], sizeof(void*), PAGE_READWRITE, &old_protect)) return false;
            original = reinterpret_cast<Fn>(vtable[index]);
            vtable[index] = replacement;
            VirtualProtect(&vtable[index], sizeof(void*), old_protect, &old_protect);
            return true;
        }

        // Creates a throwaway swap chain only to learn where DXGI keeps Present, then patches those slots.
        void install_hook_thread()
        {
            using CreateFactoryFn = HRESULT(WINAPI*)(REFIID, void**);
            using CreateDeviceFn = HRESULT(WINAPI*)(IUnknown*, D3D_FEATURE_LEVEL, REFIID, void**);
            auto create_factory = module_function<CreateFactoryFn>(L"dxgi.dll", "CreateDXGIFactory1");
            auto create_device = module_function<CreateDeviceFn>(L"d3d12.dll", "D3D12CreateDevice");
            if (!create_factory || !create_device)
            {
                HSVR_LOG("Hook: DXGI or D3D12 not available");
                return;
            }
            WNDCLASSEXW wc{};
            wc.cbSize = sizeof(wc);
            wc.lpfnWndProc = DefWindowProcW;
            wc.hInstance = GetModuleHandleW(nullptr);
            wc.lpszClassName = L"HalfSwordVRProbeWindow";
            RegisterClassExW(&wc);
            HWND window = CreateWindowExW(0, wc.lpszClassName, L"", WS_OVERLAPPEDWINDOW, 0, 0, 64, 64, nullptr, nullptr, wc.hInstance, nullptr);
            if (!window)
            {
                HSVR_LOG("Hook: could not create the probe window");
                return;
            }
            bool ok = false;
            {
                Com<IDXGIFactory2> factory;
                Com<ID3D12Device> device;
                Com<ID3D12CommandQueue> queue;
                Com<IDXGISwapChain1> swapchain;
                HRESULT hr = create_factory(IID_PPV_ARGS(factory.put()));
                if (SUCCEEDED(hr)) hr = create_device(nullptr, D3D_FEATURE_LEVEL_11_0, IID_PPV_ARGS(device.put()));
                if (SUCCEEDED(hr))
                {
                    D3D12_COMMAND_QUEUE_DESC qd{};
                    qd.Type = D3D12_COMMAND_LIST_TYPE_DIRECT;
                    hr = device->CreateCommandQueue(&qd, IID_PPV_ARGS(queue.put()));
                }
                if (SUCCEEDED(hr))
                {
                    DXGI_SWAP_CHAIN_DESC1 sd{};
                    sd.Width = 64;
                    sd.Height = 64;
                    sd.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
                    sd.SampleDesc.Count = 1;
                    sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
                    sd.BufferCount = 2;
                    sd.SwapEffect = DXGI_SWAP_EFFECT_FLIP_DISCARD;
                    hr = factory->CreateSwapChainForHwnd(queue.get(), window, &sd, nullptr, nullptr, swapchain.put());
                }
                if (SUCCEEDED(hr) && swapchain)
                {
                    void** vtable = *reinterpret_cast<void***>(swapchain.get());
                    // ID3D12CommandQueue::ExecuteCommandLists is slot 10 of the queue's function table.
                    void** queue_vtable = *reinterpret_cast<void***>(queue.get());
                    ok = patch_vtable_slot(queue_vtable, 10, reinterpret_cast<void*>(&hooked_execute), g_original_execute);
                    if (!ok) HSVR_LOG("Hook: could not hook the command queue");
                    ok = ok && patch_vtable_slot(vtable, 8, reinterpret_cast<void*>(&hooked_present), g_original_present);
                    if (ok) patch_vtable_slot(vtable, 22, reinterpret_cast<void*>(&hooked_present1), g_original_present1);
                }
                else
                {
                    HSVR_LOG("Hook: probe swap chain failed 0x%08X", (unsigned)hr);
                }
            }
            DestroyWindow(window);
            UnregisterClassW(wc.lpszClassName, wc.hInstance);
            HSVR_LOG(ok ? "Hook: Present and the command queue hooked" : "Hook: could not hook Present");
        }
    } // namespace

    bool vr_start()
    {
        Config cfg = config();
        if (!cfg.enabled)
        {
            HSVR_LOG("VR is switched off in HalfSwordVR.ini (enabled=0)");
            return false;
        }
        if (cfg.dev_fake_headset)
        {
            g_fake_headset = true;
            shared().eye_width.store(1280);
            shared().eye_height.store(1280);
            FramePose pose;
            fake_pose(pose);
            {
                std::lock_guard<std::mutex> lock(g_pipe.mutex);
                g_pipe.last_latched = pose;
            }
            shared().xr_ready.store(true);
            HSVR_LOG("TEST MODE: pretend headset (dev_fake_headset=1). No SteamVR is used; both eye views show in the game window.");
            return true;
        }
        if (cfg.require_steamvr_running && !process_running(L"vrserver.exe") && !process_running(L"vrmonitor.exe"))
        {
            HSVR_LOG("SteamVR is not running, so the game starts flat. Start SteamVR first to play in VR.");
            return false;
        }
        if (!xr_load_runtime(xr)) return false;

        if (xr.EnumerateInstanceExtensionProperties)
        {
            uint32_t count = 0;
            xr.EnumerateInstanceExtensionProperties(nullptr, 0, &count, nullptr);
            std::vector<XrExtensionProperties> exts(count, {XR_TYPE_EXTENSION_PROPERTIES});
            xr.EnumerateInstanceExtensionProperties(nullptr, count, &count, exts.data());
            bool has_d3d12 = false;
            for (auto& e : exts)
            {
                if (strcmp(e.extensionName, XR_KHR_D3D12_ENABLE_EXTENSION_NAME) == 0) has_d3d12 = true;
            }
            if (!has_d3d12)
            {
                HSVR_LOG("OpenXR: runtime does not offer Direct3D 12 support");
                return false;
            }
        }

        const char* extensions[] = {XR_KHR_D3D12_ENABLE_EXTENSION_NAME};
        XrInstanceCreateInfo ici{XR_TYPE_INSTANCE_CREATE_INFO};
        strncpy(ici.applicationInfo.applicationName, "Half Sword VR", XR_MAX_APPLICATION_NAME_SIZE - 1);
        ici.applicationInfo.applicationVersion = 1;
        strncpy(ici.applicationInfo.engineName, "HalfSwordVR mod", XR_MAX_ENGINE_NAME_SIZE - 1);
        ici.applicationInfo.apiVersion = XR_MAKE_VERSION(1, 0, 0);
        ici.enabledExtensionCount = 1;
        ici.enabledExtensionNames = extensions;
        XrResult r = xr.CreateInstance(&ici, &g_instance);
        if (XR_FAILED(r))
        {
            HSVR_LOG("OpenXR: xrCreateInstance failed (%d)", (int)r);
            return false;
        }
        if (!xr_resolve_functions(xr, g_instance)) return false;

        XrInstanceProperties ip{XR_TYPE_INSTANCE_PROPERTIES};
        if (XR_SUCCEEDED(xr.GetInstanceProperties(g_instance, &ip)))
        {
            HSVR_LOG("OpenXR: runtime %s %u.%u.%u", ip.runtimeName, (unsigned)XR_VERSION_MAJOR(ip.runtimeVersion),
                     (unsigned)XR_VERSION_MINOR(ip.runtimeVersion), (unsigned)XR_VERSION_PATCH(ip.runtimeVersion));
        }
        XrSystemGetInfo sgi{XR_TYPE_SYSTEM_GET_INFO};
        sgi.formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;
        r = xr.GetSystem(g_instance, &sgi, &g_system);
        if (XR_FAILED(r))
        {
            HSVR_LOG("OpenXR: no headset found (%s). Is the headset connected to SteamVR?", xr_result_name(r));
            return false;
        }
        XrSystemProperties sp{XR_TYPE_SYSTEM_PROPERTIES};
        if (XR_SUCCEEDED(xr.GetSystemProperties(g_instance, g_system, &sp))) HSVR_LOG("OpenXR: headset %s", sp.systemName);

        uint32_t view_count = 0;
        xr.EnumerateViewConfigurationViews(g_instance, g_system, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, 0, &view_count, nullptr);
        std::vector<XrViewConfigurationView> views(view_count, {XR_TYPE_VIEW_CONFIGURATION_VIEW});
        r = xr.EnumerateViewConfigurationViews(g_instance, g_system, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, view_count, &view_count, views.data());
        if (XR_FAILED(r) || view_count < 2)
        {
            HSVR_LOG("OpenXR: could not read the headset's view sizes (%s)", xr_result_name(r));
            return false;
        }
        int rec_w = (int)views[0].recommendedImageRectWidth;
        int rec_h = (int)views[0].recommendedImageRectHeight;
        // Multiples of 8 keep the engine's scene buffers aligned with what we ask for.
        int w = std::max(256, ((int)(rec_w * cfg.render_scale) + 4) / 8 * 8);
        int h = std::max(256, ((int)(rec_h * cfg.render_scale) + 4) / 8 * 8);
        w = std::min<int>(w, (int)views[0].maxImageRectWidth / 8 * 8);
        h = std::min<int>(h, (int)views[0].maxImageRectHeight / 8 * 8);
        shared().eye_width.store(w);
        shared().eye_height.store(h);
        HSVR_LOG("OpenXR: headset recommends %dx%d per eye; rendering %dx%d per eye (render_scale %.2f)", rec_w, rec_h, w, h, cfg.render_scale);
        g_vr_available = true;
        return true;
    }

    void vr_install_present_hook()
    {
        if (g_fake_headset) return;
        std::thread(install_hook_thread).detach();
    }

    bool vr_latch_frame_pose(FramePose& out)
    {
        FramePose pose;
        {
            std::lock_guard<std::mutex> lock(g_pipe.mutex);
            pose = g_pipe.last_latched;
        }
        if (g_fake_headset)
        {
            fake_pose(pose);
        }
        else if (shared().xr_running.load())
        {
            XrTime time = g_frame_time.load() + g_lead_ns.load();
            locate_views(time, pose);
        }
        if (!pose.valid) return false;
        {
            std::lock_guard<std::mutex> lock(g_pipe.mutex);
            pose.sequence = ++g_pipe.sequence;
            pose.presents_done = g_presents_done.load();
            g_pipe.last_latched = pose;
            g_pipe.game_queue.push_back(pose);
            while (g_pipe.game_queue.size() > (g_fake_headset ? 1u : 8u))
            {
                g_pipe.game_queue.pop_front();
                ++g_pipe.dropped_game;
            }
        }
        out = pose;
        return true;
    }

    bool vr_locate_hands(XrHandRaw out[2])
    {
        if (config().dev_fake_hands)
        {
            // Pretend controllers for tests without anyone holding them.
            float head[3];
            float yaw;
            {
                std::lock_guard<std::mutex> lock(g_recenter_mutex);
                head[0] = g_recenter_xr_pos[0];
                head[1] = g_recenter_xr_pos[1];
                head[2] = g_recenter_xr_pos[2];
                yaw = g_recenter_xr_yaw;
            }
            double t = now_seconds();
            for (int h = 0; h < 2; ++h)
            {
                float side = h ? 1.0f : -1.0f;
                float ox = 0.22f * side + 0.08f * (float)std::sin(t * 1.3);
                float oy = -0.30f + 0.08f * (float)std::cos(t * 1.3);
                float oz = -0.35f;
                float tilt_deg = 40.0f + 30.0f * (float)std::sin(t * 0.7);
                float tilt = tilt_deg * kDegToRad;
                // Turn the offset by the recentred heading (about +Y).
                float cy = std::cos(yaw), sy = std::sin(yaw);
                XrHandRaw& hand = out[h];
                hand.valid = hand.position_tracked = true;
                hand.px = head[0] + cy * ox + sy * oz;
                hand.py = head[1] + oy;
                hand.pz = head[2] - sy * ox + cy * oz;
                // Heading about +Y, then the tilt about the hand's +X.
                float hy = yaw * 0.5f, hx = tilt * 0.5f;
                hand.qx = std::cos(hy) * std::sin(hx);
                hand.qy = std::sin(hy) * std::cos(hx);
                hand.qz = -std::sin(hy) * std::sin(hx);
                hand.qw = std::cos(hy) * std::cos(hx);
                hand.trigger = hand.squeeze = 0.0f;
            }
            return true;
        }

        XrSession session = g_session.load();
        XrSpace local = g_local_space.load();
        if (g_fake_headset || !g_hands.ready.load() || !shared().xr_running.load() || session == XR_NULL_HANDLE || local == XR_NULL_HANDLE) return false;
        XrActiveActionSet active{g_hands.set, XR_NULL_PATH};
        XrActionsSyncInfo sync{XR_TYPE_ACTIONS_SYNC_INFO};
        sync.countActiveActionSets = 1;
        sync.activeActionSets = &active;
        XrResult r = xr.SyncActions(session, &sync);
        if (XR_FAILED(r)) HSVR_LOG_EVERY(10.0, "OpenXR: xrSyncActions: %s", xr_result_name(r));
        if (g_hands.profile_changed.exchange(false)) log_interaction_profiles(session);
        XrTime time = g_frame_time.load() + g_lead_ns.load();
        bool any = false;
        bool pose_active[2]{false, false};
        for (int h = 0; h < 2; ++h)
        {
            XrHandRaw& hand = g_hands.last[h];
            XrActionStateGetInfo info{XR_TYPE_ACTION_STATE_GET_INFO};
            info.action = g_hands.grip_pose;
            info.subactionPath = g_hands.hand_path[h];
            XrActionStatePose pose_state{XR_TYPE_ACTION_STATE_POSE};
            bool active_pose = XR_SUCCEEDED(xr.GetActionStatePose(session, &info, &pose_state)) && pose_state.isActive;
            pose_active[h] = active_pose;
            XrSpaceLocation loc{XR_TYPE_SPACE_LOCATION};
            if (active_pose && XR_SUCCEEDED(xr.LocateSpace(g_hands.grip_space[h], local, time, &loc)) &&
                (loc.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT) && (loc.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT))
            {
                hand.valid = true;
                hand.position_tracked = (loc.locationFlags & XR_SPACE_LOCATION_POSITION_TRACKED_BIT) != 0;
                hand.px = loc.pose.position.x;
                hand.py = loc.pose.position.y;
                hand.pz = loc.pose.position.z;
                hand.qx = loc.pose.orientation.x;
                hand.qy = loc.pose.orientation.y;
                hand.qz = loc.pose.orientation.z;
                hand.qw = loc.pose.orientation.w;
            }
            else
            {
                hand.valid = false;
            }
            hand.trigger = read_float_action(session, g_hands.trigger, h);
            hand.squeeze = read_float_action(session, g_hands.squeeze, h);
            out[h] = hand;
            any = any || hand.valid;
        }
        static bool was_any = false;
        if (!any)
        {
            HSVR_LOG_EVERY(5.0, "OpenXR: hands not tracked (pose action active L %d R %d)", pose_active[0], pose_active[1]);
        }
        else if (!was_any)
        {
            HSVR_LOG("OpenXR: hands tracked again");
        }
        was_any = any;
        return any;
    }


    Recenter vr_recenter()
    {
        std::lock_guard<std::mutex> lock(g_recenter_mutex);
        return g_recenter;
    }

    void vr_set_recenter_from(const FramePose& pose)
    {
        if (!pose.valid) return;
        const XrEyeRaw& a = pose.eye[0];
        const XrEyeRaw& b = pose.eye[1];
        float cx = (a.px + b.px) * 0.5f, cy = (a.py + b.py) * 0.5f, cz = (a.pz + b.pz) * 0.5f;
        Quat q = xr_to_ue_rotation(a.qx, a.qy, a.qz, a.qw);
        // Heading in OpenXR terms: angle about +Y, positive to the left.
        Vec3 fwd_ue = q.rotate({1, 0, 0}); // Unreal forward = OpenXR -Z, Unreal right = OpenXR +X
        float xr_yaw = (float)std::atan2(-fwd_ue.y, fwd_ue.x);
        std::lock_guard<std::mutex> lock(g_recenter_mutex);
        g_recenter.valid = true;
        g_recenter.position_cm = xr_to_ue_position(cx, cy, cz, 100.0);
        g_recenter.yaw_deg = quat_yaw_deg(q);
        g_recenter_xr_pos[0] = cx;
        g_recenter_xr_pos[1] = cy;
        g_recenter_xr_pos[2] = cz;
        g_recenter_xr_yaw = xr_yaw;
        HSVR_LOG("Recentered: head at (%.2f, %.2f, %.2f) m, heading %.0f deg", cx, cy, cz, g_recenter.yaw_deg);
    }

    void vr_render_thread_view_rect(int view_index, int min_x, int min_y, int max_x, int max_y)
    {
        if (view_index < 0 || view_index > 1) return;
        std::lock_guard<std::mutex> lock(g_pipe.mutex);
        if (!g_pipe.separate_mode) return;
        if (!g_pipe.rects_started)
        {
            g_pipe.rects_started = true;
            g_pipe.rect_valid[0] = g_pipe.rect_valid[1] = false;
        }
        g_pipe.rect[view_index][0] = min_x;
        g_pipe.rect[view_index][1] = min_y;
        g_pipe.rect[view_index][2] = max_x;
        g_pipe.rect[view_index][3] = max_y;
        g_pipe.rect_valid[view_index] = max_x > min_x && max_y > min_y;
    }

    // The pose is not attached here: the presenting thread pairs it (see take_pose_for_present).
    void vr_render_thread_frame_done(ID3D12Resource* scene)
    {
        std::lock_guard<std::mutex> lock(g_pipe.mutex);
        if (!g_pipe.rects_started) g_pipe.rect_valid[0] = g_pipe.rect_valid[1] = false; // no per-view callback this frame
        g_pipe.rects_started = false;
        PresentItem item;
        for (int e = 0; e < 2; ++e)
        {
            item.rect_valid[e] = g_pipe.rect_valid[e];
            for (int k = 0; k < 4; ++k) item.rect[e][k] = g_pipe.rect[e][k];
        }
        item.scene.assign(scene);
        g_pipe.present_queue.push_back(std::move(item));
        while (g_pipe.present_queue.size() > 4)
        {
            g_pipe.present_queue.pop_front();
            ++g_pipe.dropped_present;
        }
    }

    void vr_set_separate_target_mode(bool separate)
    {
        std::lock_guard<std::mutex> lock(g_pipe.mutex);
        g_pipe.separate_mode = separate;
        g_pipe.present_queue.clear();
        g_pipe.rects_started = false;
    }

    void vr_render_thread_target_reset()
    {
        std::lock_guard<std::mutex> lock(g_pipe.mutex);
        g_pipe.present_queue.clear();
        g_pipe.rects_started = false;
    }
} // namespace hsvr