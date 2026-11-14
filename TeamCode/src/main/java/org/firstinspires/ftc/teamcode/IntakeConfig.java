package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

/**
 * Every intake setting, exposed to the Slothboard (the Dairy fork of FTC Dashboard) as live
 * configuration variables.
 *
 * Static fields on a @Config class are the contract: the dashboard writes the field, the intake
 * reads it on its next periodic(). The AutoTuner measures beam polarity and both roller
 * directions and writes them back here, so what the dashboard shows is what the robot is using.
 *
 * The powers and times below are the values the robot ships with. The AutoTuner cannot measure
 * them - they depend on how your robot feeds and on game strategy - so they stay human choices,
 * which is exactly what the dashboard is for.
 */
@Config
public class IntakeConfig {
    // ---- motor powers (-1.0 to 1.0) ----
    public static double intakePower = 0.8;      // speed while collecting balls
    public static double fullPower = 0.2;        // slow hold once full (too high = balls jam/squish)
    public static double reversePower = -0.8;    // speed when spitting everything out
    public static double ejectPower = -0.8;      // lower roller backwards to spit out an extra ball
    public static double shootPower = 1.0;       // lower it if balls hit the flywheel too fast to shoot well

    // ---- door servo positions (0.0 to 1.0) ----
    public static double doorOpen = 0.5;         // position that lets balls into the flywheel
    public static double doorClosed = 0.0;       // position that blocks the flywheel

    // ---- counting / timing ----
    public static int maxBalls = 4;              // how many balls the robot can hold
    public static double countCooldownSeconds = 0.2;   // raise if one ball counts twice (holes in the balls)
    public static double ejectTimeSeconds = 0.5;       // long enough for the extra ball to fully leave

    // ---- hardware behaviour (measured by AutoTuner phase 7) ----
    public static boolean beamActiveLow = true;  // most beam-break sensors read LOW when blocked
    public static boolean lowerReversed = true;  // flips if the lower roller spins the wrong way
    public static boolean upperReversed = false; // flips if the upper roller spins the wrong way
}
