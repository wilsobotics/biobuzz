package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

/**
 * Every drive value the AutoTuner adjusts, exposed to the Slothboard (the Dairy fork of FTC
 * Dashboard) as live configuration variables.
 *
 * The fields are static because that is what the dashboard writes to: editing a value in the
 * browser changes the field itself and the running OpMode reads it on its next loop, with no
 * redeploy. The numbers here are the shipping defaults; AutoTuner overwrites them with whatever
 * it measures, and they keep that value for the rest of the app session until the defaults below
 * are updated to match (paste the results, then restart the app to confirm).
 *
 * Naming note: the dashboard treats these as tunable constants, so try not to rename them -
 * a renamed field shows up in the browser as a new variable with its default value.
 */
@Config
public class DriveConfig {
    public static double kpTranslational = 0.0035;                  // power per mm of position error
    public static double kpHeading = 1.25;                          // power per radian of heading error
    public static double maxHoldPower = 0.60;                       // ceiling on any one wheel while holding
    public static double minHoldPower = 0.06;                       // friction floor; set 0 to disable
    public static double holdPositionDeadbandMm = 8.0;
    public static double holdHeadingDeadbandRad = Math.toRadians(1.5);
    public static double joystickEngageDeadband = 0.08;             // stick must pass this to count as input
    public static double joystickReleaseDeadband = 0.04;            // and drop below this to let go

    // Pod counting direction: "forward" means the reading increases as the robot moves forward
    // for X, or left for Y. Measured by AutoTuner phase 1, applied in initialiseDriver(), and
    // written back here so the dashboard always shows what the robot is actually using.
    public static boolean xPodForward = true;
    public static boolean yPodForward = true;
}
