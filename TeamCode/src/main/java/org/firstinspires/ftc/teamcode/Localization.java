package org.firstinspires.ftc.teamcode;

import android.content.Context;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Where the robot is on the field. Owns the goBILDA Pinpoint, the pod configuration and the pose,
 * and is the only class that talks to the odometry computer.
 *
 * <h2>Frame and units</h2>
 * Poses are the {@link FieldMap} field frame: millimetres, radians, origin at the field centre,
 * +X toward the red wall, +Y toward the audience, heading counter-clockwise. Zero heading means
 * the robot faces +X. That frame exists because {@link #initialise} hands the Pinpoint a start
 * pose from FieldMap — before that call the Pinpoint's own frame is arbitrary.
 *
 * <h2>What to edit</h2>
 * <ul>
 *   <li>{@link Config#podOffsetXmm} / {@link Config#podOffsetYmm} - measured pod offsets in mm.
 *       X pod = sideways from the tracking point (+ left of centre), Y pod = forward of it.</li>
 *   <li>{@link Config#deviceName} - must match the robot configuration.</li>
 *   <li>{@link Config#xPodForward} / {@link Config#yPodForward} - which way each pod counts.
 *       The AutoTuner discovers these and stores them, so leave them alone once tuned.</li>
 * </ul>
 * Everything else (the field frame, start poses, zones) is in FieldMap, and everything about how
 * the robot drives from a pose is in TeleOpDriver.
 */
public class Localization {

    /** Editable configuration. Pod directions are overwritten by stored tuning at construction. */
    public static class Config {
        /** Name of the Pinpoint in the robot configuration. */
        public String deviceName = "odo";

        // TODO(robot): -84 / -168 are copied from the SDK's SensorGoBildaPinpoint sample, which
        // says they are tuned for goBILDA's 3110-0002-0001. These are YOUR pod offsets in mm:
        // X pod = sideways from the tracking point (+ left of centre), Y pod = forward of it.
        // Measure them; wrong offsets skew every position the drive and the hold brake use.
        public double podOffsetXmm = -84.0;
        public double podOffsetYmm = -168.0;

        /** False if a pod counts down when the robot moves forward / left. */
        public boolean xPodForward = true;
        public boolean yPodForward = true;
    }

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final Config config;
    private final Context appContext;

    private GoBildaPinpointDriver odo;
    private boolean configLoadedFromStore = false;

    public Localization(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, new Config());
    }

    public Localization(HardwareMap hardwareMap, Telemetry telemetry, Config config) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.config = config;
        this.appContext = hardwareMap.appContext;
        // Pod directions are a tuning result; offsets are not.
        this.configLoadedFromStore = TuneStore.loadLocalization(appContext, config);
    }

    /**
     * Brings the Pinpoint up and anchors its frame at the given field pose.
     *
     * @param startPose      where the robot is standing, from FieldMap.startPose(...)
     * @param announceToTelemetry true to print the odometry diagnostics during init
     */
    public void initialise(Pose2D startPose, boolean announceToTelemetry) {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, config.deviceName);
        odo.setOffsets(config.podOffsetXmm, config.podOffsetYmm, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        applyPodDirections();

        odo.resetPosAndIMU();
        setPose(startPose);

        if (announceToTelemetry) {
            telemetry.addData("Status", "Initialised");
            telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
            telemetry.addData("Y Offset", odo.getYOffset(DistanceUnit.MM));
            telemetry.addData("Device Version Number:", odo.getDeviceVersion());
            telemetry.addData("Device Scalar", odo.getYawScalar());
            telemetry.addData("Odo Status: ", odo.getDeviceStatus());
            telemetry.addData("Pods", (config.xPodForward ? "X FWD" : "X REV")
                    + ", " + (config.yPodForward ? "Y FWD" : "Y REV")
                    + (configLoadedFromStore ? " (stored)" : ""));
            telemetry.update();
        }
    }

    // ------------------------------------------------------------------
    // Pose
    // ------------------------------------------------------------------

    /** Pulls a fresh pose off the Pinpoint. Call once per loop before reading the pose. */
    public void update() {
        odo.update();
    }

    /** Last pose read from the Pinpoint; does not touch the bus. */
    public Pose2D getPose() {
        return odo.getPosition();
    }

    /** Refreshes the Pinpoint, then returns the pose. */
    public Pose2D readPose() {
        update();
        return getPose();
    }

    public double getHeadingRadians() {
        return getPose().getHeading(AngleUnit.RADIANS);
    }

    /** True while the Pinpoint is reporting READY; the heading is meaningless otherwise. */
    public boolean isReady() {
        return getStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
    }

    public GoBildaPinpointDriver.DeviceStatus getStatus() {
        return odo.getDeviceStatus();
    }

    /**
     * Re-anchors the frame at an explicit pose without touching the hardware, so "wherever the
     * robot stands right now" becomes that pose.
     */
    public void setPose(Pose2D pose) {
        odo.setPosition(pose);
        odo.update();
    }

    public void resetPoseToOrigin() {
        setPose(FieldMap.pose(0, 0, 0));
    }

    /**
     * Zeroes the heading only. X/Y and the IMU calibration are kept, so the robot does not need to
     * be standing still and does not lose where it is on the field. After this call heading 0
     * means "the way the robot faces now", so the pose stops matching FieldMap until the next
     * {@link #setPose} or {@link #initialise}.
     */
    public void resetHeading() {
        odo.setHeading(0, AngleUnit.RADIANS);
        // Refresh the cached pose so the next read really sees heading 0 rather than the old one.
        odo.update();
    }

    // ------------------------------------------------------------------
    // Pod configuration
    // ------------------------------------------------------------------

    /**
     * Pod counting direction. "Forward" means "increases when the robot moves forward / left";
     * if the robot drives forward but X falls, flip the X pod rather than the motors.
     */
    public void setPodDirections(boolean xForward, boolean yForward) {
        config.xPodForward = xForward;
        config.yPodForward = yForward;
        applyPodDirections();
    }

    public boolean isXPodForward() {
        return config.xPodForward;
    }

    public boolean isYPodForward() {
        return config.yPodForward;
    }

    public Config getConfig() {
        return config;
    }

    /** Persists the learned pod directions so the next run starts with them. */
    public boolean saveConfig() {
        return TuneStore.saveLocalization(appContext, config);
    }

    /** True if the pod directions came from the Control Hub rather than the compiled defaults. */
    public boolean isConfigLoadedFromStore() {
        return configLoadedFromStore;
    }

    private void applyPodDirections() {
        odo.setEncoderDirections(
                config.xPodForward ? GoBildaPinpointDriver.EncoderDirection.FORWARD
                        : GoBildaPinpointDriver.EncoderDirection.REVERSED,
                config.yPodForward ? GoBildaPinpointDriver.EncoderDirection.FORWARD
                        : GoBildaPinpointDriver.EncoderDirection.REVERSED);
    }
}
