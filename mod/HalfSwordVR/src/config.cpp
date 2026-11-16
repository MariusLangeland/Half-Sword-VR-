#include "config.hpp"

#include "log.hpp"

#include <windows.h>

#include <cstdio>
#include <cstdlib>
#include <map>
#include <mutex>
#include <string>

namespace hsvr
{
    static std::mutex g_cfg_mutex;
    static Config g_cfg;
    static FILETIME g_cfg_time{};
    static double g_last_check = -100.0;

    static std::wstring ini_path()
    {
        return mod_dir() + L"HalfSwordVR.ini";
    }

    static std::string trim(const std::string& s)
    {
        size_t a = s.find_first_not_of(" \t\r\n");
        if (a == std::string::npos) return {};
        size_t b = s.find_last_not_of(" \t\r\n");
        return s.substr(a, b - a + 1);
    }

    static void write_default_file()
    {
        FILE* f = _wfopen(ini_path().c_str(), L"w");
        if (!f) return;
        Config d;
        fprintf(f,
                "; Half Sword VR settings. Edit and save; the mod re-reads this file while the game runs.\n"
                "enabled=%d\n"
                "; Only start VR if SteamVR is already running when the game starts.\n"
                "require_steamvr_running=%d\n"
                "; Per-eye resolution as a fraction of what the headset asks for (1.0 = full).\n"
                "render_scale=%.2f\n"
                "world_scale=%.2f\n"
                "; head = view from the character's head, camera = the game's own camera position.\n"
                "anchor=%s\n"
                "head_bone=%s\n"
                "; Straight ahead follows: aim (where you aim with the mouse) or camera (the game's camera).\n"
                "heading=%s\n"
                "eye_forward_cm=%.1f\n"
                "eye_up_cm=%.1f\n"
                "anchor_smoothing=%.3f\n"
                "near_clip_cm=%.1f\n"
                "gamma_mode=%d\n"
                "ui_panel=%d\n"
                "; 0 = panel always shown (the menu-panel key hides it); 1 = only while a menu is up.\n"
                "ui_panel_auto=%d\n"
                "ui_distance_m=%.2f\n"
                "ui_width_m=%.2f\n"
                "ui_height_offset_m=%.2f\n"
                "; Opacity of menus/HUD on the panel = brightness x ui_opacity_gain.\n"
                "ui_opacity_gain=%.1f\n"
                "; Dark sheet behind the panel while a menu is up: 0 = none, 1 = solid.\n"
                "ui_menu_backdrop=%.2f\n"
                "; 1 = also use the window's own alpha for the panel (coarse; off by default).\n"
                "ui_use_alpha=%d\n"
                "; Show the left eye's picture in the game window on the monitor.\n"
                "mirror_window=%d\n"
                "; Keys are Windows virtual key codes: 0x24 = Home (recenter), 0x23 = End (show/hide menu panel),\n"
                "; 0x2D = Insert (switch between first-person and the game's own third-person camera).\n"
                "recenter_key=0x%X\n"
                "toggle_ui_key=0x%X\n"
                "toggle_view_key=0x%X\n"
                "; 0x22 = Page Down: flip between full-resolution mode and the fallback while playing.\n"
                "toggle_mode_key=0x%X\n"
                "predict_frames=%.1f\n"
                "; Head positions older than this many finished pictures are not used (2 is right for this game; -1 = off).\n"
                "pose_max_age=%d\n"
                "; 1 = full headset resolution with menus on a floating panel, 0 = fallback through the game window.\n"
                "separate_render_target=%d\n"
                "; Picture tuning (-1 = leave the game's own value): sharpen 0 to 1, screen_percentage 50 to 100.\n"
                "sharpen=%.2f\n"
                "screen_percentage=%.0f\n"
                "; Extra engine console commands, separated by semicolons (for tuning; normally empty).\n"
                "extra_commands=%s\n"
                "; 1 = keep the clouds behind buildings in both eyes (works around an engine fault).\n"
                "fix_clouds=%d\n"
                "; Which cloud mode replaces the faulty one: 2 (thin far things stay clean) or 0.\n"
                "cloud_mode=%d\n"
                "; Bounce lighting: one light sample per this many pixels. The game uses 64, which flickers\n"
                "; in the headset. 32 = twice as dense, 16 = four times, 8 = eight times (each slower), 0 = leave the game's value.\n"
                "light_samples=%d\n"
                "; 1 = keep the game at normal speed while the headset is asleep.\n"
                "idle_helper=%d\n"
                "log_poses=%d\n"
                "; Record in the log what the keyboard, mouse pointer and any gamepad are doing.\n"
                "log_input=%d\n"
                "; Save troubleshooting pictures every this many seconds into the captures folder (0 = off).\n"
                "capture_seconds=%.0f\n"
                "stereo_slot_offset=%d\n"
                "dev_fake_headset=%d\n"
                "dev_fake_hands=%d\n",
                d.enabled, d.require_steamvr_running, d.render_scale, d.world_scale, d.anchor.c_str(), d.head_bone.c_str(), d.heading.c_str(),
                d.eye_forward_cm, d.eye_up_cm, d.anchor_smoothing, d.near_clip_cm, d.gamma_mode, d.ui_panel, d.ui_panel_auto,
                d.ui_distance_m, d.ui_width_m, d.ui_height_offset_m, d.ui_opacity_gain, d.ui_menu_backdrop, d.ui_use_alpha, d.mirror_window, (unsigned)d.recenter_key, (unsigned)d.toggle_ui_key, (unsigned)d.toggle_view_key, (unsigned)d.toggle_mode_key,
                d.predict_frames, d.pose_max_age, d.separate_render_target, d.sharpen, d.screen_percentage, d.extra_commands.c_str(), d.fix_clouds, d.cloud_mode, d.light_samples, d.idle_helper, d.log_poses, d.log_input, d.capture_seconds, d.stereo_slot_offset, d.dev_fake_headset, d.dev_fake_hands);
        fprintf(f,
                "; ---- Motion controls: the controllers move the character's hands ----\n"
                "motion_arms=%d\n"
                "arm_right=%d\n"
                "arm_left=%d\n"
                "; 0x21 = Page Up: switch motion arms on and off while playing.\n"
                "toggle_arms_key=0x%X\n"
                "; 1 = the game's own aim follows the sword (swing speed, stamina, body turning).\n"
                "aim_sync=%d\n"
                "aim_scale=%.2f\n"
                "aim_invert_y=%d\n"
                "; Where the elbow points: a mix of down, out to the side and back.\n"
                "elbow_down=%.2f\n"
                "elbow_out=%.2f\n"
                "elbow_back=%.2f\n"
                "; Share of the hand's roll taken by the forearm (0 = all in the wrist).\n"
                "forearm_twist_share=%.2f\n"
                "; Slide the sword along the handle relative to the controller, cm (+ = towards the blade).\n"
                "grip_shift_cm=%.1f\n"
                "; Extra turn of the hands relative to the controllers, degrees.\n"
                "hand_pitch=%.1f\n"
                "hand_yaw=%.1f\n"
                "hand_roll=%.1f\n"
                "log_arms=%d\n",
                d.motion_arms, d.arm_right, d.arm_left, (unsigned)d.toggle_arms_key, d.aim_sync, d.aim_scale, d.aim_invert_y, d.elbow_down, d.elbow_out, d.elbow_back, d.forearm_twist_share, d.grip_shift_cm, d.hand_pitch, d.hand_yaw, d.hand_roll, d.log_arms);
        fprintf(f,
                "; ---- Debug view ----\n"
                "debug_draw=%d\n"
                "; ---- Driver pose solver ----\n"
                "pose_system=%d\n"
                "pose_hand_rotation=%d\n"
                "pose_reach_bones=%d\n"
                "pose_reach_max=%.2f\n"
                "pose_reach_knee=%.2f\n"
                "pose_reach_smoothing=%.2f\n"
                "pose_reach_spine_share=%.2f\n"
                "pose_arm_scale=%.2f\n"
                "pose_head=%d\n"
                "pose_head_bones=%d\n"
                "; 1 = the left hand joins the sword when both hands come together with a grip held.\n"
                "two_handed=%d\n"
                "two_handed_offset_cm=%.1f\n"
                "two_handed_distance_cm=%.1f\n",
                d.debug_draw, d.pose_system, d.pose_hand_rotation, d.pose_reach_bones, d.pose_reach_max, d.pose_reach_knee, d.pose_reach_smoothing,
                d.pose_reach_spine_share, d.pose_arm_scale, d.pose_head, d.pose_head_bones, d.two_handed, d.two_handed_offset_cm,
                d.two_handed_distance_cm);
        fclose(f);
    }

    Config config()
    {
        std::lock_guard<std::mutex> lock(g_cfg_mutex);
        return g_cfg;
    }

    bool config_reload_if_changed(bool force)
    {
        double t = now_seconds();
        {
            std::lock_guard<std::mutex> lock(g_cfg_mutex);
            if (!force && t - g_last_check < 1.0) return false;
            g_last_check = t;
        }
        WIN32_FILE_ATTRIBUTE_DATA info{};
        if (!GetFileAttributesExW(ini_path().c_str(), GetFileExInfoStandard, &info))
        {
            write_default_file();
            if (!GetFileAttributesExW(ini_path().c_str(), GetFileExInfoStandard, &info)) return false;
        }
        {
            std::lock_guard<std::mutex> lock(g_cfg_mutex);
            if (!force && CompareFileTime(&info.ftLastWriteTime, &g_cfg_time) == 0) return false;
            g_cfg_time = info.ftLastWriteTime;
        }
        FILE* f = _wfopen(ini_path().c_str(), L"r");
        if (!f) return false;
        std::map<std::string, std::string> kv;
        char line[512];
        while (fgets(line, sizeof(line), f))
        {
            std::string s = trim(line);
            if (s.empty() || s[0] == ';' || s[0] == '#' || s[0] == '[') continue;
            size_t eq = s.find('=');
            if (eq == std::string::npos) continue;
            kv[trim(s.substr(0, eq))] = trim(s.substr(eq + 1));
        }
        fclose(f);

        Config c;
        auto num = [&](const char* key, double& out) {
            auto it = kv.find(key);
            if (it != kv.end() && !it->second.empty()) out = strtod(it->second.c_str(), nullptr);
        };
        auto integer = [&](const char* key, int& out) {
            auto it = kv.find(key);
            if (it != kv.end() && !it->second.empty()) out = (int)strtol(it->second.c_str(), nullptr, 0);
        };
        auto flag = [&](const char* key, bool& out) {
            int v = out ? 1 : 0;
            integer(key, v);
            out = v != 0;
        };
        auto text = [&](const char* key, std::string& out) {
            auto it = kv.find(key);
            if (it != kv.end() && !it->second.empty()) out = it->second;
        };
        flag("enabled", c.enabled);
        flag("require_steamvr_running", c.require_steamvr_running);
        num("render_scale", c.render_scale);
        num("world_scale", c.world_scale);
        text("anchor", c.anchor);
        text("head_bone", c.head_bone);
        text("heading", c.heading);
        num("eye_forward_cm", c.eye_forward_cm);
        num("eye_up_cm", c.eye_up_cm);
        num("anchor_smoothing", c.anchor_smoothing);
        num("near_clip_cm", c.near_clip_cm);
        integer("gamma_mode", c.gamma_mode);
        flag("ui_panel", c.ui_panel);
        flag("ui_panel_auto", c.ui_panel_auto);
        num("ui_distance_m", c.ui_distance_m);
        num("ui_width_m", c.ui_width_m);
        num("ui_height_offset_m", c.ui_height_offset_m);
        num("ui_opacity_gain", c.ui_opacity_gain);
        num("ui_menu_backdrop", c.ui_menu_backdrop);
        flag("ui_use_alpha", c.ui_use_alpha);
        flag("mirror_window", c.mirror_window);
        integer("recenter_key", c.recenter_key);
        integer("toggle_ui_key", c.toggle_ui_key);
        integer("toggle_view_key", c.toggle_view_key);
        integer("toggle_mode_key", c.toggle_mode_key);
        num("predict_frames", c.predict_frames);
        integer("pose_max_age", c.pose_max_age);
        flag("separate_render_target", c.separate_render_target);
        num("sharpen", c.sharpen);
        num("screen_percentage", c.screen_percentage);
        text("extra_commands", c.extra_commands);
        flag("fix_clouds", c.fix_clouds);
        integer("cloud_mode", c.cloud_mode);
        integer("light_samples", c.light_samples);
        flag("wipe_standins", c.wipe_standins);
        flag("log_sliders", c.log_sliders);
        flag("idle_helper", c.idle_helper);
        flag("motion_arms", c.motion_arms);
        flag("arm_right", c.arm_right);
        flag("arm_left", c.arm_left);
        integer("toggle_arms_key", c.toggle_arms_key);
        flag("aim_sync", c.aim_sync);
        num("aim_scale", c.aim_scale);
        flag("aim_invert_y", c.aim_invert_y);
        num("elbow_down", c.elbow_down);
        num("elbow_out", c.elbow_out);
        num("elbow_back", c.elbow_back);
        num("forearm_twist_share", c.forearm_twist_share);
        num("reach_knee", c.reach_knee);
        num("reach_max", c.reach_max);
        num("shoulder_reach_cm", c.shoulder_reach_cm);
        num("assist_seconds", c.assist_seconds);
        num("assist_max_cm", c.assist_max_cm);
        num("assist_max_deg", c.assist_max_deg);
        num("wrist_bend_limit", c.wrist_bend_limit);
        num("elbow_stiffness", c.elbow_stiffness);
        num("twist_limit", c.twist_limit);
        num("twist_clamp", c.twist_clamp);
        num("elbow_swing_max", c.elbow_swing_max);
        num("grip_shift_cm", c.grip_shift_cm);
        num("hand_pitch", c.hand_pitch);
        num("hand_yaw", c.hand_yaw);
        num("hand_roll", c.hand_roll);
        flag("log_arms", c.log_arms);
        flag("debug_draw", c.debug_draw);
        flag("pose_system", c.pose_system);
        flag("pose_hand_rotation", c.pose_hand_rotation);
        integer("pose_reach_bones", c.pose_reach_bones);
        num("pose_reach_max", c.pose_reach_max);
        num("pose_reach_knee", c.pose_reach_knee);
        num("pose_reach_smoothing", c.pose_reach_smoothing);
        num("pose_reach_spine_share", c.pose_reach_spine_share);
        num("pose_arm_scale", c.pose_arm_scale);
        flag("pose_head", c.pose_head);
        integer("pose_head_bones", c.pose_head_bones);
        flag("two_handed", c.two_handed);
        num("two_handed_offset_cm", c.two_handed_offset_cm);
        num("two_handed_distance_cm", c.two_handed_distance_cm);
        flag("log_poses", c.log_poses);
        flag("log_input", c.log_input);
        num("capture_seconds", c.capture_seconds);
        flag("capture_menus_only", c.capture_menus_only);
        integer("stereo_slot_offset", c.stereo_slot_offset);
        flag("dev_fake_headset", c.dev_fake_headset);
        flag("dev_fake_still", c.dev_fake_still);
        flag("dev_fake_hands", c.dev_fake_hands);
        flag("dev_arms_any_character", c.dev_arms_any_character);
        num("dev_fake_yaw", c.dev_fake_yaw);
        num("dev_fake_pitch", c.dev_fake_pitch);
        num("dev_fake_scale", c.dev_fake_scale);
        text("dev_script", c.dev_script);
        if (c.render_scale < 0.2) c.render_scale = 0.2;
        if (c.render_scale > 2.0) c.render_scale = 2.0;
        if (c.world_scale < 0.1) c.world_scale = 0.1;
        if (c.near_clip_cm < 0.5) c.near_clip_cm = 0.5;
        {
            std::lock_guard<std::mutex> lock(g_cfg_mutex);
            g_cfg = c;
        }
        HSVR_LOG("Settings loaded: render_scale=%.2f world_scale=%.2f anchor=%s head_bone=%s eye_fwd=%.1f eye_up=%.1f smoothing=%.3f "
                 "near=%.1f gamma=%d ui_panel=%d separate_rt=%d",
                 c.render_scale, c.world_scale, c.anchor.c_str(), c.head_bone.c_str(), c.eye_forward_cm, c.eye_up_cm,
                 c.anchor_smoothing, c.near_clip_cm, c.gamma_mode, c.ui_panel, c.separate_render_target);
        return true;
    }
} // namespace hsvr