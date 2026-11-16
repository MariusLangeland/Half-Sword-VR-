// Motion controls: the controllers move the character's arms.
//
// Half Sword animates an invisible "driver" skeleton (DriverSkeleton, ABP_Willie) and the visible,
// physically simulated body (CharacterMesh0) copies that pose each frame with a CopyPoseFromMesh
// node; the body's muscles then pull the real arm bodies towards it. This file bends the driver
// skeleton's arm bones so that each hand sits where the player's controller is, just before the
// body copies the pose. Strength, stamina, injuries and weapon weight stay the game's own.
// See docs/phase3-motion-controls.md.
#pragma once

#include "vrmath.hpp"

#include <cstdint>
#include <vector>

namespace RC::Unreal
{
    class UObject;
}

namespace hsvr
{
    // Once per engine tick on the game thread, before the world ticks. `pawn` is the player's
    // character (may be null), `menu_up` true while a menu or pause screen is showing.
    void arm_game_tick(RC::Unreal::UObject* pawn, bool menu_up);

    // Flips motion arms on or off while playing (a key).
    void arm_toggle();

    // Shows the game's hidden driver skeleton mesh (debug_draw setting) so the pose the solver
    // produces can be seen. Works with motion arms off or a menu up.
    void arm_debug_draw();
} // namespace hsvr
