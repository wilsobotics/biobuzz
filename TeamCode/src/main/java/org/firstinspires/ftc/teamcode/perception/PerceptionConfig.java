package org.firstinspires.ftc.teamcode.perception;

import com.acmerobotics.dashboard.config.Config;

/** Every perception tunable. Editable live from FTC Dashboard; read at point of use. */
@Config("Perception")
public final class PerceptionConfig {
    private PerceptionConfig() {
    }

    // wiring
    public static String LIMELIGHT_NAME = "limelight";
    public static String TILT_SERVO_NAME = "camTilt";   // "" = fixed mount
    public static int GOAL_PIPELINE = 0;                 // AprilTag (fiducial) pipeline
    /** Ball pipelines, e.g. "1:NEURAL" or "1:COLOUR:POLLEN,2:COLOUR:RED_NECTAR,3:COLOUR:BLUE_NECTAR". */
    public static String BALL_PIPELINES = "1:NEURAL";
    public static double TILT_BALL_POS = 0.5;            // measure with the prelim bench
    public static double TILT_GOAL_POS = 0.5;

    // timing
    public static long MODE_SETTLE_MS = 250;             // servo move + pipeline switch
    public static long PIPELINE_SETTLE_MS = 100;         // pipeline switch only (colour cycling)
    public static long MAX_STALENESS_MS = 150;
    public static long POSE_HISTORY_MS = 500;

    // mount (from CAD at turret 0 deg; update when the mount is final)
    public static double CAM_FORWARD_IN = 0.0;
    public static double CAM_LEFT_IN = 0.0;
    public static double CAM_HEIGHT_IN = 12.4;
    public static double BALL_PITCH_DOWN_DEG = 25.0;     // set from the prelim bench
    public static double TURRET_BALL_YAW_DEG = 0.0;      // 180 later for the dual intake
    public static double HFOV_DEG = 54.5;
    public static double VFOV_DEG = 42.0;

    // sighting quality
    public static double MIN_CONFIDENCE = 0.25;          // junk floor only; real thresholds from capture data
    public static double GOOD_RANGE_IN = 48.0;
    public static double MAX_RANGE_IN = 120.0;
    public static double FAR_RANGE_FACTOR = 0.3;
    public static double EDGE_MARGIN_DEG = 4.0;
    public static double EDGE_FACTOR = 0.6;
    public static double ANGLE_SIGMA_DEG = 0.5;          // assumed angle noise -> position sigma
    public static double MIN_SIGMA_IN = 0.5;
    public static double TARGET_QUALITY = 0.5;
    public static double TARGET_SIGMA_IN = 2.0;
    public static double MAYBE_TWO_ASPECT = 1.6;

    // data association
    public static double GATE_SIGMAS = 3.0;
    public static double MIN_GATE_IN = 1.5;
    public static double TIGHT_GATE_DIAMETERS = 0.8;
    public static int CONFIRM_FRAMES = 2;
    public static long CANDIDATE_TIMEOUT_MS = 500;
    public static long FORGET_MS = 4000;
    public static int MISS_FRAMES_TO_REMOVE = 2;
    public static double VIEW_REMOVE_RANGE_IN = 60.0;
    /** A confirmed map ball (any class) seen within this long can hide a farther ball behind it. */
    public static long OCCLUDER_RECENT_MS = 1000;
    /** A sighting may reset maybeTwo when its sigma <= this x the sharpest sigma that set the flag. */
    public static double SHAPE_SIGMA_SLACK = 1.25;

    // pickup bookkeeping
    public static double INTAKE_REACH_IN = 14.3;         // robot centre to intake mouth (CAD)
    public static double PICKUP_SEARCH_IN = 10.0;
}
