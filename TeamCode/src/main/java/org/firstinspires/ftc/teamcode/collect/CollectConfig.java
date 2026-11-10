package org.firstinspires.ftc.teamcode.collect;

import com.acmerobotics.dashboard.config.Config;

/** Every collect/pathing tunable. Editable live from FTC Dashboard; read at point of use. */
@Config("Collect")
public final class CollectConfig {
    private CollectConfig() {
    }

    // frames
    public static double PEDRO_ROTATION_DEG = 0;   // manual frame -> Pedro frame, multiples of 90
    public static double DASH_ROTATION_DEG = 0;    // Pedro frame -> dashboard drawing

    // robot + intake (Weaver geometry)
    public static int INTAKE_CAPACITY = 4;         // only for CountingIntakeStatus; real intake reports its own
    public static double INTAKE_WIDTH_IN = 11.3;
    public static double ROBOT_LENGTH_IN = 18.0;
    public static double ROBOT_WIDTH_IN = 18.0;
    public static double CLEARANCE_IN = 2.0;
    public static int MAX_WEAVER_TARGETS = 7;      // Weaver's exact-ordering cap

    // zones
    public static double MIDLINE_MARGIN_IN = 4.0;
    public static double CUSTOM_MIN_X = 0, CUSTOM_MIN_Y = 0, CUSTOM_MAX_X = 144, CUSTOM_MAX_Y = 144;

    // run
    public static double AUTO_TIMEOUT_SEC = 20;
    public static boolean COLLECT_NECTAR = false;  // v1 robot collects pollen only; true adds own-alliance nectar to the default targets
    public static double LOOK_DISTANCE_IN = 30;
    public static long LOOK_DWELL_MS = 400;
    /** After start, an empty map waits this long for targets before searching or giving up. */
    public static long START_DWELL_MS = 400;
    public static double STICK_CANCEL = 0.1;

    // re-plan
    public static long REPLAN_COOLDOWN_MS = 300;
    public static double OFF_PATH_IN = 6.0;
    public static double OFF_PATH_DEG = 20.0;
    /** The heading part of OFF_PATH only fires once the error has lasted this long. */
    public static long OFF_PATH_HEADING_MS = 600;
    /** PATH_DONE cannot re-plan sooner than this after the last re-plan. */
    public static long PATH_DONE_MIN_MS = 100;
    public static long STALL_MS = 500;
    public static double STALL_MOVE_IN = 1.0;
    public static double POSE_JUMP_IN = 4.0;           // floor: a jump must also move at least this far
    /** A pose change is a jump (relocalisation) only above this implied speed. */
    public static double POSE_JUMP_IN_PER_S = 120.0;
    public static double BETTER_BALL_MARGIN_IN = 6.0;
    public static double REACH_TOL_IN = 6.0;
    public static long PICKUP_GRACE_MS = 600;
    public static int MISS_LIMIT = 2;
    /** How long a ball Weaver could not reach is left out of target choice before it is tried again. */
    public static long UNREACHABLE_RETRY_MS = 1500;
}
