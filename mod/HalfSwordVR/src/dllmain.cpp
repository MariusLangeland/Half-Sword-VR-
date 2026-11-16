// HalfSwordVR: native VR for Half Sword, loaded by UE4SS.
// This file is the glue to the game: it starts the headset link, installs our stereo device in
// the running engine, and tells it where the player's head is each frame.

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <map>
#include <string>
#include <vector>

#include <DynamicOutput/DynamicOutput.hpp>
#include <Mod/CppUserModBase.hpp>
#include <Unreal/FProperty.hpp>
#include <Unreal/Property/FBoolProperty.hpp>
#include <Unreal/Property/FStructProperty.hpp>
#include <Unreal/UScriptStruct.hpp>
#include <Unreal/FString.hpp>
#include <Unreal/Hooks.hpp>
#include <Unreal/NameTypes.hpp>
#include <Unreal/UClass.hpp>
#include <Unreal/UFunction.hpp>
#include <Unreal/UObject.hpp>
#include <Unreal/UObjectGlobals.hpp>

#include "arm_control.hpp"
#include "config.hpp"
#include "engine_slot.hpp"
#include "input_watch.hpp"
#include "log.hpp"
#include "stereo_device.hpp"
#include "ue_abi.hpp"
#include "vr_present.hpp"

#include <windows.h>

using namespace RC;
using namespace RC::Unreal;

namespace
{
    // ---- reflection helpers (game thread) ------------------------------------------------------
    UObject* object_property(UObject* owner, const TCHAR* name)
    {
        if (!owner) return nullptr;
        UObject** value = owner->GetValuePtrByPropertyNameInChain<UObject*>(name);
        return value ? *value : nullptr;
    }

    // Reads a yes/no property that the engine packs into a single bit.
    bool bool_property(UObject* owner, const TCHAR* name)
    {
        if (!owner) return false;
        FProperty* property = owner->GetPropertyByNameInChain(name);
        if (!property) return false;
        auto* flag = static_cast<FBoolProperty*>(property);
        const uint8_t* byte = reinterpret_cast<const uint8_t*>(owner) + property->GetOffset_Internal() + flag->GetByteOffset();
        return (*byte & flag->GetFieldMask()) != 0;
    }

    // What a widget with a "Brush" (an Image) draws: a texture, a material or a render target.
    UObject* brush_resource(UObject* widget)
    {
        if (!widget) return nullptr;
        FProperty* property = widget->GetPropertyByNameInChain(STR("Brush"));
        if (!property) return nullptr;
        UScriptStruct* brush_struct = static_cast<FStructProperty*>(property)->GetStruct();
        if (!brush_struct) return nullptr;
        for (FProperty* member : brush_struct->ForEachProperty())
        {
            if (member->GetName() != STR("ResourceObject")) continue;
            const uint8_t* brush = reinterpret_cast<const uint8_t*>(widget) + property->GetOffset_Internal();
            return *reinterpret_cast<UObject* const*>(brush + member->GetOffset_Internal());
        }
        return nullptr;
    }

    struct RawArray
    {
        void** Data;
        int32_t Num;
        int32_t Max;
    };

    UObject* g_engine = nullptr;
    bool g_vr_started = false;
    bool g_install_failed = false;
    bool g_console_done = false;
    UObject* g_logged_pawn = nullptr;

    UObject* find_player_controller()
    {
        UObject* viewport = object_property(g_engine, STR("GameViewport"));
        UObject* game_instance = object_property(viewport, STR("GameInstance"));
        if (!game_instance) return nullptr;
        RawArray* players = game_instance->GetValuePtrByPropertyNameInChain<RawArray>(STR("LocalPlayers"));
        if (!players || players->Num < 1 || !players->Data) return nullptr;
        UObject* local_player = static_cast<UObject*>(players->Data[0]);
        return object_property(local_player, STR("PlayerController"));
    }

    UObject* find_pawn_mesh(UObject** out_pawn = nullptr)
    {
        UObject* controller = find_player_controller();
        UObject* pawn = object_property(controller, STR("Pawn"));
        if (out_pawn) *out_pawn = pawn;
        if (!pawn) return nullptr;
        return object_property(pawn, STR("Mesh"));
    }

    struct SocketLocationParams
    {
        FName InSocketName;
        ue::FVector ReturnValue;
    };

    struct ActorLocationParams
    {
        ue::FVector ReturnValue;
    };

    struct BoneNameParams
    {
        int32_t BoneIndex;
        FName ReturnValue;
    };

    struct NumBonesParams
    {
        int32_t ReturnValue;
    };

    FName head_bone_name()
    {
        static std::string cached_text;
        static FName cached{};
        std::string wanted = hsvr::config().head_bone;
        if (wanted != cached_text)
        {
            cached_text = wanted;
            std::wstring wide(wanted.begin(), wanted.end());
            cached = FName(wide.c_str(), FNAME_Add);
        }
        return cached;
    }

    void log_bones_once(UObject* pawn, UObject* mesh)
    {
        if (pawn == g_logged_pawn) return;
        g_logged_pawn = pawn;
        HSVR_LOG("Player pawn: %ls", pawn->GetFullName().c_str());
        HSVR_LOG("Player mesh: %ls", mesh->GetFullName().c_str());
        UFunction* num_fn = mesh->GetFunctionByNameInChain(STR("GetNumBones"));
        UFunction* name_fn = mesh->GetFunctionByNameInChain(STR("GetBoneName"));
        if (!num_fn || !name_fn) return;
        NumBonesParams count{};
        mesh->ProcessEvent(num_fn, &count);
        std::wstring line;
        for (int32_t i = 0; i < count.ReturnValue && i < 400; ++i)
        {
            BoneNameParams p{};
            p.BoneIndex = i;
            mesh->ProcessEvent(name_fn, &p);
            line += p.ReturnValue.ToString();
            line += L" ";
            if (line.size() > 900)
            {
                HSVR_LOG("Bones: %ls", line.c_str());
                line.clear();
            }
        }
        if (!line.empty()) HSVR_LOG("Bones (%d total): %ls", count.ReturnValue, line.c_str());
    }

    // Called by the stereo device while the frame's views are being set up (game thread).
    bool provide_anchor(hsvr::PlayerAnchor& out)
    {
        if (!g_engine) return false;
        UObject* pawn = nullptr;
        UObject* mesh = find_pawn_mesh(&pawn);
        if (!pawn || !mesh) return false;
        log_bones_once(pawn, mesh);

        UFunction* socket_fn = mesh->GetFunctionByNameInChain(STR("GetSocketLocation"));
        UFunction* actor_fn = pawn->GetFunctionByNameInChain(STR("K2_GetActorLocation"));
        if (!socket_fn || !actor_fn) return false;
        SocketLocationParams socket{};
        socket.InSocketName = head_bone_name();
        mesh->ProcessEvent(socket_fn, &socket);
        ActorLocationParams actor{};
        pawn->ProcessEvent(actor_fn, &actor);
        out.head_position = {socket.ReturnValue.X, socket.ReturnValue.Y, socket.ReturnValue.Z};
        out.actor_position = {actor.ReturnValue.X, actor.ReturnValue.Y, actor.ReturnValue.Z};
        out.has_head = true;
        if (UObject* controller = find_player_controller())
        {
            if (ue::FRotator* rotation = controller->GetValuePtrByPropertyNameInChain<ue::FRotator>(STR("ControlRotation")))
            {
                out.has_heading = true;
                out.heading_yaw = rotation->Yaw;
            }
        }
        return true;
    }

    struct ConsoleCommandParams
    {
        UObject* WorldContextObject;
        FString Command;
        UObject* SpecificPlayer;
    };

    void run_console_command(const TCHAR* command)
    {
        static UFunction* fn = UObjectGlobals::StaticFindObject<UFunction*>(nullptr, nullptr, STR("/Script/Engine.KismetSystemLibrary:ExecuteConsoleCommand"));
        static UObject* cdo = UObjectGlobals::StaticFindObject<UObject*>(nullptr, nullptr, STR("/Script/Engine.Default__KismetSystemLibrary"));
        UObject* viewport = object_property(g_engine, STR("GameViewport"));
        if (!fn || !cdo || !viewport) return;
        ConsoleCommandParams params{viewport, FString(command), nullptr};
        cdo->ProcessEvent(fn, &params);
    }

    void run_console_command(const std::string& command)
    {
        std::wstring wide(command.begin(), command.end());
        run_console_command(wide.c_str());
    }

    struct ConsoleVariableParams
    {
        FString VariableName;
        float ReturnValue;
    };

    // The value of an engine setting, as a number. A setting this game does not have reads as 0.
    float console_variable(const TCHAR* name)
    {
        static UFunction* fn =
                UObjectGlobals::StaticFindObject<UFunction*>(nullptr, nullptr, STR("/Script/Engine.KismetSystemLibrary:GetConsoleVariableFloatValue"));
        static UObject* cdo = UObjectGlobals::StaticFindObject<UObject*>(nullptr, nullptr, STR("/Script/Engine.Default__KismetSystemLibrary"));
        if (!fn || !cdo) return -999.0f;
        ConsoleVariableParams params{FString(name), 0.0f};
        cdo->ProcessEvent(fn, &params);
        return params.ReturnValue;
    }

    // Writes the game's picture settings to the log, so that sharpness can be tuned from facts.
    void log_graphics(const char* reason)
    {
        static const TCHAR* const names[] = {
                STR("r.ScreenPercentage"),
                STR("r.ScreenPercentage.Default"),
                STR("r.ScreenPercentage.Default.VR.Mode"),
                STR("r.SecondaryScreenPercentage.GameViewport"),
                STR("r.AntiAliasingMethod"),
                STR("r.TemporalAA.Upsampling"),
                STR("r.TSR.History.ScreenPercentage"),
                STR("r.Tonemapper.Sharpen"),
                STR("r.Tonemapper.Quality"),
                STR("r.SceneColorFringeQuality"),
                STR("r.Tonemapper.GrainQuantization"),
                STR("r.FilmGrain"),
                STR("r.BloomQuality"),
                STR("r.LensFlareQuality"),
                STR("r.MotionBlurQuality"),
                STR("r.DepthOfFieldQuality"),
                STR("r.VolumetricCloud"),
                STR("r.VolumetricRenderTarget"),
                STR("r.VolumetricRenderTarget.Mode"),
                STR("r.VolumetricRenderTarget.UpsamplingMode"),
                STR("Slate.CopyBackbufferToSlatePostRenderTargets"),
                STR("r.TSR.ShadingRejection.Flickering"),
                STR("r.TSR.History.UpdateQuality"),
                STR("r.DynamicRes.OperationMode"),
                STR("r.NGX.DLSS.Enable"),
                STR("r.NGX.DLAA.Enable"),
                STR("r.Streamline.DLSSG.Enable"),
                STR("r.FidelityFX.FSR3.Enabled"),
                STR("r.FidelityFX.FSR3.QualityMode"),
                STR("r.FidelityFX.FI.Enabled"),
                STR("r.DynamicGlobalIlluminationMethod"),
                STR("r.ReflectionMethod"),
                STR("r.Lumen.DiffuseIndirect.Allow"),
                STR("r.Lumen.Reflections.Allow"),
                STR("r.Lumen.HardwareRayTracing"),
                STR("r.Lumen.TraceMeshSDFs.Allow"),
                STR("r.Lumen.ScreenProbeGather.DownsampleFactor"),
                STR("r.Lumen.ScreenProbeGather.TracingOctahedronResolution"),
                STR("r.Lumen.ScreenProbeGather.IrradianceFormat"),
                STR("r.Lumen.ScreenProbeGather.StochasticInterpolation"),
                STR("r.Lumen.ScreenProbeGather.FullResolutionJitterWidth"),
                STR("r.Lumen.ScreenProbeGather.FixedJitterIndex"),
                STR("r.Lumen.ScreenProbeGather.ScreenTraces"),
                STR("r.Lumen.ScreenProbeGather.ScreenTraces.HZBTraversal.FullResDepth"),
                STR("r.Lumen.ScreenProbeGather.ShortRangeAO"),
                STR("r.Lumen.ScreenProbeGather.Temporal.MaxFramesAccumulated"),
                STR("r.Lumen.ScreenProbeGather.TemporalFilterProbes"),
                STR("r.Lumen.ScreenProbeGather.TwoSidedFoliageBackfaceDiffuse"),
                STR("r.Lumen.ScreenProbeGather.RadianceCache.ProbeResolution"),
                STR("r.LumenScene.Radiosity.ProbeSpacing"),
                STR("r.DistanceFieldAO"),
                STR("r.AOQuality"),
                STR("r.Shadow.Virtual.Enable"),
                STR("r.MaxAnisotropy"),
                STR("r.MipMapLODBias"),
                STR("r.VSync"),
                STR("t.MaxFPS"),
                STR("sg.ResolutionQuality"),
                STR("sg.ViewDistanceQuality"),
                STR("sg.AntiAliasingQuality"),
                STR("sg.ShadowQuality"),
                STR("sg.GlobalIlluminationQuality"),
                STR("sg.ReflectionQuality"),
                STR("sg.PostProcessQuality"),
                STR("sg.TextureQuality"),
                STR("sg.EffectsQuality"),
                STR("sg.FoliageQuality"),
                STR("sg.ShadingQuality"),
        };
        HSVR_LOG("Graphics (%s), the engine's picture settings (one the game does not have reads 0):", reason);
        for (const TCHAR* name : names) HSVR_LOG("  Graphics: %ls = %.3f", name, console_variable(name));
        HSVR_LOG("Graphics: end of list");
    }

    // Passes the picture tuning from the settings file on to the engine, when it is set and
    // whenever it changes (the file can be edited while the game runs).
    void apply_picture_settings(const hsvr::Config& cfg)
    {
        static double applied_sharpen = -1.0, applied_percentage = -1.0;
        static std::string applied_commands;
        char command[96];
        if (cfg.sharpen >= 0.0 && cfg.sharpen != applied_sharpen)
        {
            applied_sharpen = cfg.sharpen;
            snprintf(command, sizeof(command), "r.Tonemapper.Sharpen %.2f", cfg.sharpen);
            run_console_command(std::string(command));
            HSVR_LOG("Picture: %s", command);
        }
        if (cfg.screen_percentage >= 0.0 && cfg.screen_percentage != applied_percentage)
        {
            applied_percentage = cfg.screen_percentage;
            snprintf(command, sizeof(command), "r.ScreenPercentage %.0f", cfg.screen_percentage);
            run_console_command(std::string(command));
            HSVR_LOG("Picture: %s", command);
        }
        if (cfg.extra_commands != applied_commands)
        {
            applied_commands = cfg.extra_commands;
            size_t start = 0;
            while (start < applied_commands.size())
            {
                size_t end = applied_commands.find(';', start);
                if (end == std::string::npos) end = applied_commands.size();
                std::string one = applied_commands.substr(start, end - start);
                size_t first = one.find_first_not_of(" \t");
                size_t last = one.find_last_not_of(" \t");
                if (first != std::string::npos)
                {
                    one = one.substr(first, last - first + 1);
                    run_console_command(one);
                    HSVR_LOG("Picture: extra command: %s", one.c_str());
                }
                start = end + 1;
            }
        }
    }

    // Unreal 5.4 draws volumetric clouds per view in one of four modes. In mode 1 (and 3) it looks
    // up the scene's depth for a view at the wrong place when that view does not start at the
    // left edge of the picture, which is the case for our second eye: the clouds then land in
    // front of everything in that eye. Modes 0 and 2 are right. Half Sword switches to mode 1
    // when an outdoor arena loads, so the mode is checked every frame and replaced. Mode 2 is the
    // default replacement: mode 0 lays a half-size cloud picture over the scene, and cloud then
    // lands on thin far things against the sky (a steeple, leaves), which flicker.
    void keep_clouds_correct(const hsvr::Config& cfg)
    {
        if (!cfg.fix_clouds) return;
        // Modes 1 and 3 are the faulty ones; 0 and 2 are right in both eyes (ini cloud_mode).
        float target = cfg.cloud_mode == 2 ? 2.0f : 0.0f;
        static float last_set = -1.0f;
        float mode = console_variable(STR("r.VolumetricRenderTarget.Mode"));
        if (mode < -900.0f || mode == target) return;
        // Leave a good mode the game chose itself alone; replace a faulty one, and one we set
        // ourselves when the setting has since changed.
        if (mode != 1.0f && mode != 3.0f && mode != last_set) return;
        run_console_command(target == 2.0f ? STR("r.VolumetricRenderTarget.Mode 2") : STR("r.VolumetricRenderTarget.Mode 0"));
        last_set = target;
        static int lines_left = 20;
        if (lines_left > 0)
        {
            --lines_left;
            HSVR_LOG("Clouds: cloud mode was %g, set to %g so that both eyes show them behind the scenery (now reads %g)", (double)mode, (double)target,
                     (double)console_variable(STR("r.VolumetricRenderTarget.Mode")));
        }
    }

    // Half Sword runs the engine's bounce lighting (Lumen global illumination) with one light
    // sample per 64 pixels. On a monitor that is about 30 samples across the picture; in the
    // headset each eye is drawn only about 1000 pixels wide, which leaves about 16. Far thin
    // things against the sky (a steeple, leaves) then catch a different sample from frame to frame
    // and flicker, and plants are lit in blotches. The player found this with a key that stepped
    // through tests (since removed): lighting off stopped the flicker, twice as dense samples
    // stopped most of it and made the plants look better, and of 32, 16 and 8 he preferred 16.
    // The game sets its value again when a map loads, so it is checked every frame.
    void keep_light_samples(const hsvr::Config& cfg)
    {
        if (cfg.light_samples <= 0) return;
        float target = (float)cfg.light_samples;
        static float last_set = -1.0f;
        float value = console_variable(STR("r.Lumen.ScreenProbeGather.DownsampleFactor"));
        if (value < -900.0f || value == target) return;
        // Lower a coarser value; leave a finer one the game chose alone, but follow the setting
        // when it is changed while the game runs.
        if (value < target && value != last_set) return;
        char command[96];
        snprintf(command, sizeof(command), "r.Lumen.ScreenProbeGather.DownsampleFactor %d", cfg.light_samples);
        run_console_command(std::string(command));
        last_set = target;
        static int lines_left = 20;
        if (lines_left > 0)
        {
            --lines_left;
            HSVR_LOG("Lighting: one bounce light sample per %g pixels, set to %g so that far thin things do not flicker in the headset (now reads %g)",
                     (double)value, (double)target, (double)console_variable(STR("r.Lumen.ScreenProbeGather.DownsampleFactor")));
        }
    }

    // ---- unattended tests (ini dev_script) -------------------------------------------------------
    // Runs console commands at set times, so that a test needs no key or click.
    struct ScriptEntry
    {
        double at = 0.0;
        std::string command;
        bool done = false;
    };

    void run_dev_script(const hsvr::Config& cfg)
    {
        static bool parsed = false;
        static std::vector<ScriptEntry> entries;
        static ULONGLONG start = 0;
        if (!parsed)
        {
            parsed = true;
            start = GetTickCount64();
            size_t pos = 0;
            const std::string& text = cfg.dev_script;
            while (pos < text.size())
            {
                size_t end = text.find(';', pos);
                if (end == std::string::npos) end = text.size();
                std::string one = text.substr(pos, end - pos);
                pos = end + 1;
                size_t colon = one.find(':');
                if (colon == std::string::npos) continue;
                ScriptEntry entry;
                entry.at = atof(one.substr(0, colon).c_str());
                entry.command = one.substr(colon + 1);
                size_t first = entry.command.find_first_not_of(" \t");
                size_t last = entry.command.find_last_not_of(" \t\r\n");
                if (first == std::string::npos) continue;
                entry.command = entry.command.substr(first, last - first + 1);
                entries.push_back(entry);
            }
            if (!entries.empty()) HSVR_LOG("Script: %d timed step(s) from the settings file (dev_script)", (int)entries.size());
        }
        if (entries.empty()) return;
        double now = (double)(GetTickCount64() - start) / 1000.0;
        for (ScriptEntry& entry : entries)
        {
            if (entry.done || now < entry.at) continue;
            entry.done = true;
            SYSTEMTIME utc{};
            GetSystemTime(&utc);
            HSVR_LOG("Script: %.0f s: %s (%02d:%02d:%02d.%03d UTC)", entry.at, entry.command.c_str(), (int)utc.wHour, (int)utc.wMinute, (int)utc.wSecond,
                     (int)utc.wMilliseconds);
            run_console_command(entry.command);
            return; // one a tick
        }
    }

    // ---- installing the stereo device ------------------------------------------------------------
    // UEngine keeps `TSharedPtr<IStereoRendering> StereoRenderingDevice` followed directly by
    // `TSharedPtr<IXRTrackingSystem> XRSystem`. Neither is a reflected property, so their offset is
    // worked out from reflected neighbours (filled in from the UE 5.4.4 layout; see find_stereo_slot).
    ue::TSharedPtr<ue::IStereoRendering>* find_stereo_slot(UObject* engine)
    {
        auto* base = reinterpret_cast<uint8_t*>(engine);
        size_t offset = (size_t)hsvr::config().stereo_slot_offset;
        if (offset == 0)
        {
            // Both neighbours are reflected properties, so their offsets can be asked for by name.
            size_t begin = 0x400, end = 0x1800;
            FProperty* before = engine->GetPropertyByNameInChain(STR("SelectionHighlightIntensityBillboards"));
            FProperty* after = engine->GetPropertyByNameInChain(STR("GlobalNetTravelCount"));
            if (before && after && after->GetOffset_Internal() > before->GetOffset_Internal())
            {
                begin = (size_t)before->GetOffset_Internal() + 4;
                end = (size_t)after->GetOffset_Internal();
            }
            else
            {
                HSVR_LOG("Neighbouring engine properties not found; searching a default range");
            }
            HSVR_LOG("Searching for the stereo device slot in UEngine between 0x%zX and 0x%zX", begin, end);
            offset = hsvr::find_stereo_slot_offset(base, begin, end);
        }
        if (offset == 0) return nullptr;
        return reinterpret_cast<ue::TSharedPtr<ue::IStereoRendering>*>(base + offset);
    }

    void install_stereo_device()
    {
        auto* slot = find_stereo_slot(g_engine);
        if (!slot)
        {
            g_install_failed = true;
            HSVR_LOG("Could not find the engine's stereo device slot; VR stays off");
            return;
        }
        if (slot->Object != nullptr)
        {
            g_install_failed = true;
            HSVR_LOG("The engine already has a stereo device (%p); not replacing it. Remove -emulatestereo from the launch options.", (void*)slot->Object);
            return;
        }
        slot->ReferenceController = hsvr::stereo_device_reference_controller();
        slot->Object = static_cast<ue::IStereoRendering*>(hsvr::stereo_device_object());
        hsvr::shared().stereo_installed.store(true);
        HSVR_LOG("Stereo device installed in the engine at offset 0x%llX",
                 (unsigned long long)(reinterpret_cast<uint8_t*>(slot) - reinterpret_cast<uint8_t*>(g_engine)));
    }

    struct IsPausedParams
    {
        UObject* WorldContextObject;
        bool ReturnValue;
    };

    // ---- what the game's menus and HUD are made of (for the log) --------------------------------
    struct BoolReturnParams
    {
        bool ReturnValue;
    };

    struct ByteReturnParams
    {
        uint8_t ReturnValue;
    };

    struct IntReturnParams
    {
        int32_t ReturnValue;
    };

    struct ChildAtParams
    {
        int32_t Index;
        UObject* ReturnValue;
    };

    struct FloatParams
    {
        float Value;
    };

    // Calls `visit(widget, depth)` for a widget and everything inside it.
    template <typename Visit>
    void walk_widget_tree(UObject* widget, int depth, int& budget, Visit&& visit)
    {
        if (!widget || depth > 6 || budget <= 0) return;
        --budget;
        visit(widget, depth);
        // A widget made in the game's editor keeps its parts in a tree of its own.
        if (UObject* tree = object_property(widget, STR("WidgetTree")))
        {
            walk_widget_tree(object_property(tree, STR("RootWidget")), depth + 1, budget, visit);
        }
        UFunction* count_fn = widget->GetFunctionByNameInChain(STR("GetChildrenCount"));
        UFunction* child_fn = widget->GetFunctionByNameInChain(STR("GetChildAt"));
        if (!count_fn || !child_fn) return;
        IntReturnParams count{};
        widget->ProcessEvent(count_fn, &count);
        for (int32_t i = 0; i < count.ReturnValue && i < 64; ++i)
        {
            ChildAtParams child{};
            child.Index = i;
            widget->ProcessEvent(child_fn, &child);
            walk_widget_tree(child.ReturnValue, depth + 1, budget, visit);
        }
    }

    // Calls `visit` for every widget the game currently has on screen.
    template <typename Visit>
    int walk_screen_widgets(Visit&& visit)
    {
        std::vector<UObject*> widgets;
        UObjectGlobals::FindAllOf(STR("UserWidget"), widgets);
        int budget = 400;
        for (UObject* widget : widgets)
        {
            if (!widget) continue;
            UFunction* in_viewport_fn = widget->GetFunctionByNameInChain(STR("IsInViewport"));
            if (!in_viewport_fn) continue;
            BoolReturnParams in_viewport{};
            widget->ProcessEvent(in_viewport_fn, &in_viewport);
            if (!in_viewport.ReturnValue) continue;
            walk_widget_tree(widget, 0, budget, visit);
        }
        return (int)widgets.size();
    }

    // Visibility values are the engine's ESlateVisibility: 0 visible, 1 collapsed, 2 hidden,
    // 3 not hit-testable (self and children), 4 not hit-testable (self only).
    void log_widgets(const char* reason)
    {
        HSVR_LOG("Widgets (%s), those on screen:", reason);
        int lines = 0;
        int total = walk_screen_widgets([&lines](UObject* widget, int depth) {
            int visibility = -1;
            if (UFunction* visibility_fn = widget->GetFunctionByNameInChain(STR("GetVisibility")))
            {
                ByteReturnParams v{};
                widget->ProcessEvent(visibility_fn, &v);
                visibility = v.ReturnValue;
            }
            float opacity = -1.0f;
            if (UFunction* opacity_fn = widget->GetFunctionByNameInChain(STR("GetRenderOpacity")))
            {
                FloatParams o{};
                widget->ProcessEvent(opacity_fn, &o);
                opacity = o.Value;
            }
            ++lines;
            if (UObject* resource = brush_resource(widget))
            {
                HSVR_LOG("  %*s%ls | visibility %d, opacity %.2f | draws %ls", depth * 2, "", widget->GetFullName().c_str(), visibility, opacity,
                         resource->GetFullName().c_str());
            }
            else
            {
                HSVR_LOG("  %*s%ls | visibility %d, opacity %.2f", depth * 2, "", widget->GetFullName().c_str(), visibility, opacity);
            }
        });
        HSVR_LOG("Widgets: end of list (%d line(s), %d widgets exist)", lines, total);
    }

    // What each slider on screen is doing, written whenever it changes. A slider follows the
    // pointer only while it "holds the mouse": from a button press on it until the release.
    // Holding it with no real button down means the game missed the release.
    void log_sliders()
    {
        static std::map<UObject*, std::string> last;
        static int lines_left = 300;
        if (lines_left <= 0) return;
        bool button = (GetAsyncKeyState(VK_LBUTTON) & 0x8000) != 0;
        walk_screen_widgets([&](UObject* widget, int) {
            auto name = widget->GetFullName();
            if (name.rfind(STR("Slider "), 0) != 0) return;
            bool holds = false, hovered = false;
            float value = -1.0f;
            if (UFunction* fn = widget->GetFunctionByNameInChain(STR("HasMouseCapture")))
            {
                BoolReturnParams r{};
                widget->ProcessEvent(fn, &r);
                holds = r.ReturnValue;
            }
            if (UFunction* fn = widget->GetFunctionByNameInChain(STR("IsHovered")))
            {
                BoolReturnParams r{};
                widget->ProcessEvent(fn, &r);
                hovered = r.ReturnValue;
            }
            if (UFunction* fn = widget->GetFunctionByNameInChain(STR("GetValue")))
            {
                FloatParams r{};
                widget->ProcessEvent(fn, &r);
                value = r.Value;
            }
            char state[96];
            snprintf(state, sizeof(state), "value %.3f, holds the mouse %d, pointer over it %d, real left button down %d", (double)value, holds ? 1 : 0,
                     hovered ? 1 : 0, button ? 1 : 0);
            std::string& before = last[widget];
            if (before == state || lines_left <= 0) return;
            // A slider nobody touches is written once only.
            bool first = before.empty();
            before = state;
            if (!first || holds || hovered)
            {
                --lines_left;
                HSVR_LOG("Slider: %ls | %s", name.c_str(), state);
            }
        });
    }

    // A number that changes when a menu screen opens or closes: which widgets are on screen and
    // which of them are visible.
    uint64_t screen_signature()
    {
        uint64_t hash = 1469598103934665603ull;
        walk_screen_widgets([&hash](UObject* widget, int) {
            int visibility = -1;
            if (UFunction* visibility_fn = widget->GetFunctionByNameInChain(STR("GetVisibility")))
            {
                ByteReturnParams v{};
                widget->ProcessEvent(visibility_fn, &v);
                visibility = v.ReturnValue;
            }
            hash = (hash ^ (uint64_t)(uintptr_t)widget) * 1099511628211ull;
            hash = (hash ^ (uint64_t)(visibility + 1)) * 1099511628211ull;
        });
        return hash;
    }

    // Feeds the input record in the log (see input_watch.hpp) and tells the picture side whether
    // a menu is up. Reads only.
    void watch_input()
    {
        hsvr::InputWatchGame game;
        UObject* pawn = nullptr;
        UObject* pawn_mesh = find_pawn_mesh(&pawn);
        if (UObject* controller = find_player_controller())
        {
            game.have_player = true;
            if (ue::FRotator* rotation = controller->GetValuePtrByPropertyNameInChain<ue::FRotator>(STR("ControlRotation")))
            {
                game.view_yaw = rotation->Yaw;
            }
            game.wants_pointer = bool_property(controller, STR("bShowMouseCursor"));
            static UFunction* fn = UObjectGlobals::StaticFindObject<UFunction*>(nullptr, nullptr, STR("/Script/Engine.GameplayStatics:IsGamePaused"));
            static UObject* cdo = UObjectGlobals::StaticFindObject<UObject*>(nullptr, nullptr, STR("/Script/Engine.Default__GameplayStatics"));
            if (fn && cdo)
            {
                IsPausedParams params{controller, false};
                cdo->ProcessEvent(fn, &params);
                game.paused = params.ReturnValue;
            }
        }
        auto& shared = hsvr::shared();
        shared.game_paused.store(game.paused);
        shared.game_wants_pointer.store(game.wants_pointer);
        // No character yet means the intro or the main menu.
        shared.menu_up.store(game.paused || !pawn || !pawn_mesh);
        hsvr::input_watch_tick(game);

        // A few times per run, write down what is on screen: once early (intro), a few seconds
        // into each new character's life, and whenever the game pauses.
        if (!hsvr::config().log_input) return;
        static int lists_left = 8;
        static double due = -1.0;
        static const char* due_reason = "";
        static bool intro_done = false, was_paused = false;
        static UObject* last_pawn = nullptr;
        static double first_tick = hsvr::now_seconds();
        double now = hsvr::now_seconds();
        if (!intro_done && now - first_tick > 20.0)
        {
            intro_done = true;
            due = now;
            due_reason = "20 seconds in";
        }
        if (pawn && pawn_mesh && pawn != last_pawn)
        {
            last_pawn = pawn;
            due = now + 5.0;
            due_reason = "a few seconds after the character appeared";
        }
        if (game.paused && !was_paused)
        {
            due = now + 0.5;
            due_reason = "game paused";
        }
        was_paused = game.paused;
        if (due >= 0.0 && now >= due && lists_left > 0)
        {
            due = -1.0;
            --lists_left;
            log_widgets(due_reason);
            // The picture settings alongside the first two lists (start-up, and once in a fight).
            static int graphics_left = 2;
            if (graphics_left > 0)
            {
                --graphics_left;
                log_graphics(due_reason);
            }
        }

        // While a menu has the mouse pointer, also write the list whenever the screen changes, so
        // that every menu screen the player visits is on record (checked twice a second).
        static double last_slider_scan = 0.0;
        if (hsvr::config().log_sliders && game.wants_pointer && now - last_slider_scan >= 0.25)
        {
            last_slider_scan = now;
            log_sliders();
        }
        static double last_scan = 0.0;
        static uint64_t last_signature = 0;
        static int change_lists_left = 16;
        if (game.wants_pointer && change_lists_left > 0 && now - last_scan >= 0.5)
        {
            last_scan = now;
            uint64_t signature = screen_signature();
            if (signature != last_signature)
            {
                last_signature = signature;
                --change_lists_left;
                log_widgets("the menu screen changed");
            }
        }
    }

    bool key_pressed_once(int vk, bool& was_down)
    {
        bool down = vk > 0 && (GetAsyncKeyState(vk) & 0x8000) != 0;
        bool pressed = down && !was_down;
        was_down = down;
        return pressed;
    }

    void game_tick(UObject* engine, float delta_seconds)
    {
        g_engine = engine;
        hsvr::config_reload_if_changed();
        hsvr::stereo_begin_game_frame(delta_seconds);
        if (!g_vr_started) return;
        auto& shared = hsvr::shared();

        if (!shared.stereo_installed.load() && !g_install_failed && shared.xr_ready.load()) install_stereo_device();

        if (shared.stereo_installed.load() && !g_console_done)
        {
            g_console_done = true;
            // The headset paces the frames; the game's own limits would only add judder.
            run_console_command(STR("r.VSync 0"));
            run_console_command(STR("t.MaxFPS 0"));
            run_console_command(STR("r.MotionBlurQuality 0"));
            run_console_command(STR("r.DepthOfFieldQuality 0"));
            // Keep menus and HUD in the game window (we show that on a floating panel) instead of
            // letting the engine smear them across both eyes. Only matters while the eyes have
            // their own target.
            run_console_command(STR("Slate.DrawToVRRenderTarget 0"));
            HSVR_LOG("Applied VR console settings (vsync off, no frame cap, no motion blur, no depth of field)");
        }

        watch_input();
        {
            UObject* pawn = nullptr;
            find_pawn_mesh(&pawn);
            bool menu = shared.menu_up.load() || shared.game_wants_pointer.load();
            if (!pawn && hsvr::config().dev_arms_any_character)
            {
                // Unattended tests: when nobody plays a character, drive the arms of the first
                // fighter in the level (the hands then reach towards the camera).
                static UObject* stand_in = nullptr;
                static double next_look = 0.0;
                if (hsvr::now_seconds() >= next_look)
                {
                    next_look = hsvr::now_seconds() + 2.0;
                    std::vector<UObject*> fighters;
                    UObjectGlobals::FindAllOf(STR("Willie_BP_C"), fighters);
                    UObject* first = fighters.empty() ? nullptr : fighters[0];
                    if (first != stand_in && first) HSVR_LOG("Test: %d fighter(s) in the level; arms driven on %ls", (int)fighters.size(), first->GetFullName().c_str());
                    stand_in = first;
                }
                pawn = stand_in;
                menu = shared.game_paused.load();
            }
            hsvr::arm_game_tick(pawn, menu);
            hsvr::arm_debug_draw();
        }

        hsvr::Config cfg = hsvr::config();
        if (g_console_done)
        {
            apply_picture_settings(cfg);
            keep_clouds_correct(cfg);
            keep_light_samples(cfg);
            run_dev_script(cfg);
        }
        // The mod's keys count only while the game has the keyboard, so that Home or End typed in
        // another program does not move the view or hide the panel.
        static bool recenter_down = false, ui_down = false, view_down = false, mode_down = false, arms_down = false;
        bool arms = key_pressed_once(cfg.toggle_arms_key, arms_down);
        bool recenter = key_pressed_once(cfg.recenter_key, recenter_down);
        bool ui = key_pressed_once(cfg.toggle_ui_key, ui_down);
        bool mode = key_pressed_once(cfg.toggle_mode_key, mode_down);
        bool view = key_pressed_once(cfg.toggle_view_key, view_down);
        DWORD front_process = 0;
        if (HWND front = GetForegroundWindow()) GetWindowThreadProcessId(front, &front_process);
        if (front_process != GetCurrentProcessId()) return;
        if (arms) hsvr::arm_toggle();
        if (recenter)
        {
            shared.recenter_requested.store(true);
            HSVR_LOG("Key: recenter");
        }
        if (ui)
        {
            bool toggled = !shared.ui_toggled.load();
            shared.ui_toggled.store(toggled);
            if (cfg.ui_panel_auto)
            {
                HSVR_LOG(toggled ? "Key: menu panel always shown" : "Key: menu panel shown only while a menu is up");
            }
            else
            {
                HSVR_LOG(toggled ? "Key: menu panel hidden during play (menus still show)" : "Key: menu panel shown");
            }
        }
        if (mode)
        {
            bool separate = hsvr::stereo_separate_target();
            shared.separate_override.store(separate ? 0 : 1);
            HSVR_LOG(separate ? "Key: fallback mode through the game window" : "Key: full-resolution mode");
        }
        if (view)
        {
            bool third = !shared.third_person.load();
            shared.third_person.store(third);
            HSVR_LOG(third ? "Key: view from the game's own camera (third person)" : "Key: view from the character's head (first person)");
        }
    }
} // namespace

class HalfSwordVR : public CppUserModBase
{
  public:
    HalfSwordVR() : CppUserModBase()
    {
        ModName = STR("HalfSwordVR");
        ModVersion = STR("0.1.0");
        ModDescription = STR("Native VR for Half Sword");
        ModAuthors = STR("Half-Sword-VR contributors");
        hsvr::log_open();
        HSVR_LOG("HalfSwordVR 0.1.0 loaded");
        hsvr::config_reload_if_changed(true);
    }

    ~HalfSwordVR() override = default;

    auto on_unreal_init() -> void override
    {
        HSVR_LOG("Unreal initialised");
        g_vr_started = hsvr::vr_start();
        if (g_vr_started)
        {
            hsvr::stereo_set_anchor_provider(&provide_anchor);
            hsvr::vr_install_present_hook();
        }
        Hook::RegisterEngineTickPreCallback(
                [](auto&, UEngine* engine, float delta_seconds, bool) {
                    game_tick(reinterpret_cast<UObject*>(engine), delta_seconds);
                },
                {false, false, STR("HalfSwordVR"), STR("Tick")});
        Output::send<LogLevel::Verbose>(STR("[HalfSwordVR] started (headset link {})\n"), g_vr_started ? STR("ok") : STR("off, see HalfSwordVR.log"));
    }
};

#define HALFSWORDVR_API __declspec(dllexport)
extern "C"
{
    HALFSWORDVR_API RC::CppUserModBase* start_mod()
    {
        return new HalfSwordVR();
    }

    HALFSWORDVR_API void uninstall_mod(RC::CppUserModBase* mod)
    {
        delete mod;
    }
}
