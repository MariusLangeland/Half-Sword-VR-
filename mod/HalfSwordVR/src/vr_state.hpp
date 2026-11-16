// State shared between the game thread (stereo device), the render thread and the thread that
// presents frames (OpenXR frame loop).
#pragma once
#include "vrmath.hpp"

#include <atomic>
#include <cstdint>

namespace hsvr
{
    struct EyeFov
    {
        // Tangents of the half angles; left and down are negative.
        double left = -1, right = 1, up = 1, down = -1;
    };

    // One eye in tracking space (OpenXR LOCAL space), exactly as the runtime gave it.
    struct XrEyeRaw
    {
        float px = 0, py = 0, pz = 0;
        float qx = 0, qy = 0, qz = 0, qw = 1;
        float angle_left = -0.78f, angle_right = 0.78f, angle_up = 0.78f, angle_down = -0.78f;
    };

    // The pose a frame is rendered with. Travels with the frame from the game thread to xrEndFrame.
    struct FramePose
    {
        uint64_t sequence = 0;
        int64_t predicted_time = 0; // XrTime the pose was predicted for
        uint64_t presents_done = 0; // Presents of the game's window finished when the pose was taken
        bool valid = false;
        XrEyeRaw eye[2];
    };

    // One controller in tracking space (OpenXR LOCAL space, metres), at the grip: the centre of
    // the closed hand, -Z along the handle towards the thumb, +X to the right, +Y up.
    struct XrHandRaw
    {
        bool valid = false;            // the pose is tracked (or was, moments ago)
        bool position_tracked = false;
        float px = 0, py = 0, pz = 0;
        float qx = 0, qy = 0, qz = 0, qw = 1;
        float trigger = 0, squeeze = 0; // 0..1
    };

    struct VrShared
    {
        // Set once the OpenXR session has produced a first valid view (FOV known).
        std::atomic<bool> xr_ready{false};
        // Set while the session is running and frames are being submitted.
        std::atomic<bool> xr_running{false};
        // Per-eye render size in pixels (after render_scale).
        std::atomic<int> eye_width{0};
        std::atomic<int> eye_height{0};
        // True once our stereo device is installed in the engine.
        std::atomic<bool> stereo_installed{false};
        // Asked by the game thread (recenter key); consumed where the head pose is known.
        std::atomic<bool> recenter_requested{true};
        // Flipped by the menu-panel key. With ui_panel_auto the panel shows while a menu is up
        // and this forces it on all the time; without, the panel is always on and this hides it.
        std::atomic<bool> ui_toggled{false};
        // Told by the game thread: the game is paused / a menu is probably up (paused, or there
        // is no player character yet, as in the intro and main menu).
        std::atomic<bool> game_paused{false};
        std::atomic<bool> menu_up{true};
        // The game has asked for a mouse pointer (it does in menus, not in a fight).
        std::atomic<bool> game_wants_pointer{false};
        // View from the game's own (third-person) camera instead of the character's head.
        std::atomic<bool> third_person{false};
        // Set by a key to flip between full-resolution mode and the fallback while playing:
        // -1 = follow the settings file, 0 = fallback, 1 = full resolution.
        std::atomic<int> separate_override{-1};
        // Diagnostics.
        std::atomic<uint64_t> frames_submitted{0};
        std::atomic<uint64_t> frames_without_image{0};
    };

    VrShared& shared();
} // namespace hsvr