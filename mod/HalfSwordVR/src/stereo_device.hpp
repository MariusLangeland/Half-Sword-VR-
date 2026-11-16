// Our implementation of the engine's stereo rendering device. Once installed in the running
// engine, the engine draws a left and a right view every frame using the headset's pose and
// lens shape, which we then hand to the headset.
#pragma once
#include "vrmath.hpp"

namespace hsvr
{
    // What the game glue knows about the player this frame (game thread only).
    struct PlayerAnchor
    {
        bool has_head = false;
        Vec3 head_position;  // world, cm
        Vec3 actor_position; // world, cm
        // Where the player is aiming (the controller's heading, set by the mouse), in degrees.
        bool has_heading = false;
        double heading_yaw = 0.0;
    };
    using AnchorProvider = bool (*)(PlayerAnchor& out);
    void stereo_set_anchor_provider(AnchorProvider provider);

    // Puts a controller into the game world the same way the eyes are put there (recentering,
    // head anchor, heading, world scale), using the most recent frame's view. Game thread only.
    // `rot` is in Unreal axes: its X axis is the controller's handle direction (OpenXR -Z).
    bool stereo_tracking_to_world(float px, float py, float pz, float qx, float qy, float qz, float qw, Vec3& pos, Quat& rot);

    // Called once per engine tick on the game thread, before the frame is drawn.
    void stereo_begin_game_frame(double delta_seconds);

    // The player's head (between the eyes) in world cm from the (possibly body-anchored) camera,
    // minus the eye offsets: where the head bone should be. Independent of the head bone itself.
    bool stereo_headset_world(Vec3& out_position_cm);

    // Projects a world position (Unreal centimetres) into one eye's normalised image coordinates
    // (0..1, u right, v down). False when the point is behind the eye.
    bool stereo_world_to_eye_uv(const Vec3& world_cm, float px, float py, float pz, float qx, float qy, float qz, float qw, float angle_left,
                                float angle_right, float angle_up, float angle_down, float& u, float& v, float& depth_m);

    // True while the engine draws the eyes into their own full-size target (settings key
    // separate_render_target) rather than side by side into the game window.
    bool stereo_separate_target();

    // The device object and its shared-pointer control block, laid out the way the engine expects
    // a TSharedPtr<IStereoRendering> to look.
    void* stereo_device_object();
    void* stereo_device_reference_controller();
} // namespace hsvr
