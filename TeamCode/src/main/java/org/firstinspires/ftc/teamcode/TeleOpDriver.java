package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Mecanum drive train with Pinpoint odometry: field-centric drive plus an odometry position hold.
 *
 * This is a plain class, not an OpMode: the SDK only fills in hardwareMap, gamepad1 and
 * telemetry on the OpMode it actually runs, so they are handed in by the OpMode instead.
 *
 * Sign conventions used throughout, and the assumptions they rest on:
 *   - Field/Pinpoint frame: +X forward, +Y to the LEFT, +heading counter-clockwise.
 *   - Wheel mix: +strafe is RIGHT, +rotate is CLOCKWISE, so it takes the robot-frame vector.
 *   - Heading 0 is the field +X direction; Y resets it without touching X/Y.
 * The drive and hold maths must agree on all of the above - if a wheel is wired the other way
 * round, every direction flips together, so fix it in one place only.
 */
public class TeleOpDriver {
    //
    // All tuning values live in DriveConfig, which is a @Config class: the Slothboard writes the
    // static fields directly, so a change in the browser applies on the next loop.
    //
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final Gamepad gamepad;

    private GoBildaPinpointDriver odo;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // Hold brake state
    private boolean isHoldBrakeEnabled = false;
    private boolean isTargetPositionLocked = false;
    private double targetXmm;
    private double targetYmm;
    private double targetHeadingRad;

    // Last pose sampled from the bus. getPose() returns this, so a caller that reads twice in one
    // loop still costs a single Pinpoint read (see updateOdometry()).
    private Pose2D cachedPose = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0);

    // A stick was pushed while the hold brake was engaged; one-shot, read by consumeStickBreach().
    private boolean stickBreachLatch = false;

    // Scratch space for the per-loop hold maths, so drive() allocates nothing.
    private double holdForwardOut;
    private double holdRightOut;
    private final double[] wheelSpeeds = new double[4];

    private boolean translateInputActive = false;
    private boolean rotateInputActive = false;
    private boolean odometryReady = false;
    private boolean telemetryEnabled = true;
    private boolean xPodForward = true;
    private boolean yPodForward = true;
    private String driveMode = "FIELD";

    // Starting pose written into the Pinpoint at init. The origin by default: that is the frame
    // the hold-brake targets and the AutoTuner assume. Call setStartingPose() for a match tile.
    private double startXmm = 0;
    private double startYmm = 0;
    private double startHeadingRad = 0;

    public TeleOpDriver(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.gamepad = gamepad;
    }

    /** Where the robot actually starts, in field coordinates. Call before initialiseDriver(). */
    public void setStartingPose(double xMm, double yMm, double headingRad) {
        startXmm = xMm; startYmm = yMm; startHeadingRad = headingRad;
    }

    public void initialiseDriver() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        backLeft = hardwareMap.get(DcMotor.class, "BackLeftMotor");
        backRight = hardwareMap.get(DcMotor.class, "BackRightMotor");
        frontLeft = hardwareMap.get(DcMotor.class, "FrontLeftMotor");
        frontRight = hardwareMap.get(DcMotor.class, "FrontRightMotor");

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Odometry Computer Config
        // TODO(robot): -84 / -168 are copied from the SDK's SensorGoBildaPinpoint sample, which
        // says they are tuned for goBILDA's 3110-0002-0001. These are YOUR pod offsets in mm:
        // X pod = sideways from the tracking point (+ left of centre), Y pod = forward of it.
        // Measure them; wrong offsets skew every position the hold brake and field drive use.
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        setPodDirections(DriveConfig.xPodForward, DriveConfig.yPodForward);

        // Odometry starting pose. resetPosAndIMU() calibrates the IMU and zeroes the counters;
        // without it the heading at match start is whatever the hub last believed, and every
        // field-centric stick direction and hold-brake correction inherits that error.
        odo.resetPosAndIMU();
        odo.setPosition(new Pose2D(DistanceUnit.MM, startXmm, startYmm, AngleUnit.RADIANS, startHeadingRad));

        // Telemetry
        if (telemetryEnabled) {
            telemetry.addData("Status", "Initialised");
            telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
            telemetry.addData("Y Offset", odo.getYOffset(DistanceUnit.MM));
            telemetry.addData("Device Version Number:", odo.getDeviceVersion());
            telemetry.addData("Device Scalar", odo.getYawScalar());
            telemetry.addData("Odo Status: ", odo.getDeviceStatus());
            telemetry.update();
        }
    }

    public void setTelemetryEnabled(boolean enabled) {
        telemetryEnabled = enabled;
    }

    // ------------------------------------------------------------------
    // Hold brake controls
    // ------------------------------------------------------------------

    public boolean isHoldBrakeEnabled() {
        return isHoldBrakeEnabled;
    }

    public void setHoldBrake(boolean enabled) {
        isHoldBrakeEnabled = enabled;
        if (!enabled) {
            clearTarget();
        }
    }

    public void toggleHoldBrake() {
        setHoldBrake(!isHoldBrakeEnabled);
    }

    /**
     * True if a stick was pushed while the hold brake was engaged; clears on read. A stick input
     * drops the hold, which silently turns a tuning trial into a manually driven robot, so the
     * AutoTuner uses this to throw the trial away instead of scoring it.
     */
    public boolean consumeStickBreach() {
        boolean breach = stickBreachLatch;
        stickBreachLatch = false;
        return breach;
    }

    /** Locks the hold onto a specific pose. Used by the AutoTuner to set up a known error. */
    public void setHoldTarget(double xMm, double yMm, double headingRad) {
        targetXmm = xMm;
        targetYmm = yMm;
        targetHeadingRad = headingRad;
        isTargetPositionLocked = true;
    }

    private void clearTarget() {
        isTargetPositionLocked = false;
        targetXmm = 0;
        targetYmm = 0;
        targetHeadingRad = 0;
    }

    /**
     * Zeroes the field-centric heading only. X/Y and the IMU calibration are kept, so the robot
     * does not need to be standing still and does not lose where it is on the field.
     */
    public void resetHeading() {
        odo.setHeading(0, AngleUnit.RADIANS);
        // Refresh the cached pose so the next read really sees heading 0 rather than the old one.
        updateOdometry();
        if (isTargetPositionLocked) {
            // Keep holding the same spot; only the heading reference moved.
            targetHeadingRad = 0;
        }
    }

    // ------------------------------------------------------------------
    // Odometry access (used by AutoTuner and diagnostics)
    // ------------------------------------------------------------------

    public GoBildaPinpointDriver.DeviceStatus getOdometryStatus() {
        return odo.getDeviceStatus();
    }

    public boolean isOdometryReady() {
        return odo.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
    }

    /**
     * The pose from the most recent bus read. Cheap, and safe to call repeatedly - but it is a
     * cache: call readPose() or drive() first if you need a fresh sample. There is deliberately
     * only one method that touches the bus, because two Pinpoint reads in one loop cost real time.
     */
    public Pose2D getPose() {
        return cachedPose;
    }

    /** Reads the Pinpoint once and returns the fresh pose. */
    public Pose2D readPose() {
        updateOdometry();
        return cachedPose;
    }

    private void updateOdometry() {
        odo.update();
        cachedPose = odo.getPosition();
        odometryReady = odo.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
    }

    public void resetPoseToOrigin() {
        // The hold target is expressed in the old pose frame, so a reset has to drop it too;
        // otherwise the next drive() chases a target that no longer means anything and lunges.
        clearTarget();
        odo.setPosition(new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0));
        updateOdometry();
    }

    /**
     * Pod counting direction. "Forward" here means "increases when the robot moves forward /
     * left"; if the robot drives forward but X falls, flip the X pod rather than the motors.
     */
    public void setPodDirections(boolean xForward, boolean yForward) {
        xPodForward = xForward;
        yPodForward = yForward;
        // Mirror into the config so the dashboard shows what the robot is actually using.
        DriveConfig.xPodForward = xForward;
        DriveConfig.yPodForward = yForward;
        odo.setEncoderDirections(
                xPodForward ? GoBildaPinpointDriver.EncoderDirection.FORWARD
                            : GoBildaPinpointDriver.EncoderDirection.REVERSED,
                yPodForward ? GoBildaPinpointDriver.EncoderDirection.FORWARD
                            : GoBildaPinpointDriver.EncoderDirection.REVERSED);

        // Flipping a pod inverts the sign of every computed pose, so zero the pose in the same
        // call. A caller that forgets would see the robot jump by its current position, and no
        // one should have to remember that.
        resetPoseToOrigin();
    }

    public boolean isXPodForward() {
        return xPodForward;
    }

    public boolean isYPodForward() {
        return yPodForward;
    }

    // ------------------------------------------------------------------
    // Drive
    // ------------------------------------------------------------------

    /** Runs the wheels at these robot-frame powers, bypassing the gamepad and the hold. */
    public void driveOpenLoop(double forward, double strafe, double rotate) {
        updateOdometry();
        applyWheelPowers(forward, strafe, rotate);
    }

    public void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private double normalizeAngle(double radians) {
        double angle = radians;
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private double clamp(double value, double limit) {
        return Math.max(-limit, Math.min(limit, value));
    }

    /**
     * Latch for one stick axis pair: engages above the engage threshold and only releases once
     * the stick is back below the release threshold.
     */
    private boolean updateInputLatch(boolean active, double magnitude) {
        if (magnitude > DriveConfig.joystickEngageDeadband) return true;
        if (magnitude < DriveConfig.joystickReleaseDeadband) return false;
        return active;
    }

    /**
     * Translational hold correction for a robot-frame error, lifted to a power that can actually
     * overcome static friction.
     */
    private void holdTranslation(double errorForward, double errorRight) {
        holdForwardOut = 0;
        holdRightOut = 0;

        double translationMagnitude = Math.hypot(errorForward, errorRight);
        if (translationMagnitude < DriveConfig.holdPositionDeadbandMm) {
            return;
        }

        double forward = errorForward * DriveConfig.kpTranslational;
        double right = errorRight * DriveConfig.kpTranslational;

        double powerMagnitude = Math.hypot(forward, right);
        if (powerMagnitude <= 0) {
            return;
        }
        if (powerMagnitude < DriveConfig.minHoldPower) {
            // A P-only output smaller than this just hums against static friction and the robot
            // parks short of the target, so lift the vector to the floor.
            double scale = DriveConfig.minHoldPower / powerMagnitude;
            forward *= scale;
            right *= scale;
        }

        double floor = Math.min(DriveConfig.minHoldPower, DriveConfig.maxHoldPower);
        if (powerMagnitude < floor) {
            double scale = floor / powerMagnitude;
            forward *= scale;
            right *= scale;
        }

        holdForwardOut = forward;
        holdRightOut = right;
    }

    /** Heading hold correction for a CCW-positive heading error. */
    private double holdHeading(double deltaHeading) {
        if (Math.abs(deltaHeading) < DriveConfig.holdHeadingDeadbandRad) {
            return 0;
        }
        // The wheel mix treats +rotate as clockwise, so a positive (CCW) error needs negative power.
        return clamp(-deltaHeading * DriveConfig.kpHeading, DriveConfig.maxHoldPower);
    }

    /** Wheel mix plus normalisation. Inputs are in the robot frame. */
    private void applyWheelPowers(double forward, double strafe, double rotate) {
        wheelSpeeds[0] = forward + strafe + rotate; // front left
        wheelSpeeds[1] = forward - strafe - rotate; // front right
        wheelSpeeds[2] = forward - strafe + rotate; // back left
        wheelSpeeds[3] = forward + strafe - rotate; // back right

        double maxPower = 1.0;
        for (double wheelSpeed : wheelSpeeds) {
            maxPower = Math.max(maxPower, Math.abs(wheelSpeed));
        }

        frontLeft.setPower(wheelSpeeds[0] / maxPower);
        frontRight.setPower(wheelSpeeds[1] / maxPower);
        backLeft.setPower(wheelSpeeds[2] / maxPower);
        backRight.setPower(wheelSpeeds[3] / maxPower);
    }

    public void drive(double speedMultiplier) {
        updateOdometry();

        Pose2D pos = cachedPose;

        // Without trustworthy odometry the heading is meaningless, so drive robot-centric
        // (heading 0 makes the field-to-robot rotation the identity) and refuse to hold.
        double heading = odometryReady ? pos.getHeading(AngleUnit.RADIANS) : 0;

        double forward = -gamepad.left_stick_y; // Inverted Y-axis
        double strafe = gamepad.left_stick_x;
        double rotate = gamepad.right_stick_x;

        translateInputActive = updateInputLatch(translateInputActive,
                Math.max(Math.abs(forward), Math.abs(strafe)));
        rotateInputActive = updateInputLatch(rotateInputActive, Math.abs(rotate));

        // Any real stick input while the brake is engaged drops the hold. Latch that, so a caller
        // (the AutoTuner) can tell a legitimate trial from one a stray thumb just ruined.
        if (isHoldBrakeEnabled && (translateInputActive || rotateInputActive)) {
            stickBreachLatch = true;
        }

        boolean holdActive = isHoldBrakeEnabled && odometryReady && !translateInputActive;

        double driveForward;
        double driveStrafe;
        double driveRotate;

        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        if (holdActive) {
            if (!isTargetPositionLocked) {
                targetXmm = pos.getX(DistanceUnit.MM);
                targetYmm = pos.getY(DistanceUnit.MM);
                targetHeadingRad = heading;
                isTargetPositionLocked = true;
            }

            double deltaX = targetXmm - pos.getX(DistanceUnit.MM);
            double deltaY = targetYmm - pos.getY(DistanceUnit.MM);
            double deltaHeading = normalizeAngle(targetHeadingRad - heading);

            // Rotate the field-frame error into the robot frame before feeding the wheel mix.
            // globalStrafe is RIGHT-positive, hence the negative of the robot's left axis.
            double errorForward = deltaX * cos + deltaY * sin;
            double errorRight = deltaX * sin - deltaY * cos;

            holdTranslation(errorForward, errorRight);
            driveForward = holdForwardOut;
            driveStrafe = holdRightOut;

            if (rotateInputActive) {
                // Turn on the stick while still holding position, and let the held heading follow
                // the robot so releasing the stick does not spin back to the old heading.
                driveRotate = rotate * speedMultiplier;
                targetHeadingRad = heading;
            } else {
                driveRotate = holdHeading(deltaHeading);
            }

            // The wheel mix is forward +/- strafe +/- rotate, so bounding the sum bounds every
            // wheel. Capping each axis separately would let a combined correction exceed the cap
            // and then be silently scaled down by the normaliser.
            double holdTotal = Math.abs(driveForward) + Math.abs(driveStrafe) + Math.abs(driveRotate);
            if (holdTotal > DriveConfig.maxHoldPower) {
                double scale = DriveConfig.maxHoldPower / holdTotal;
                driveForward *= scale;
                driveStrafe *= scale;
                driveRotate *= scale;
            }

            driveMode = rotateInputActive ? "HOLD+TURN" : "HOLD";
        } else {
            clearTarget();

            // Field-centric drive: rotate the stick vector from the field frame into the robot
            // frame. Stick forward is field +X; strafe is right-positive to match the wheels.
            driveForward = (forward * cos - strafe * sin) * speedMultiplier;
            driveStrafe = (forward * sin + strafe * cos) * speedMultiplier;
            driveRotate = rotate * speedMultiplier;

            driveMode = odometryReady ? "FIELD" : "ROBOT (odo not ready)";
        }

        applyWheelPowers(driveForward, driveStrafe, driveRotate);

        if (!telemetryEnabled) {
            return;
        }

        // Telemetry
        telemetry.addData("Mode: ", driveMode);
        telemetry.addData("Odo Status: ", odo.getDeviceStatus());
        telemetry.addData("Robot X (mm): ", pos.getX(DistanceUnit.MM));
        telemetry.addData("Robot Y (mm): ", pos.getY(DistanceUnit.MM));
        telemetry.addData("Robot Heading: ", odo.getHeading(AngleUnit.RADIANS));
        telemetry.addData("Forward Power: ", driveForward);
        telemetry.addData("Strafe Power: ", driveStrafe);
        telemetry.addData("Rotate Power: ", driveRotate);
        telemetry.addData("Hold Brake Enabled: ", isHoldBrakeEnabled);
        telemetry.addData("Target Pose Locked: ", isTargetPositionLocked);
        if (isTargetPositionLocked) {
            telemetry.addData("Target X (mm): ", targetXmm);
            telemetry.addData("Target Y (mm): ", targetYmm);
            telemetry.addData("Target Heading: ", targetHeadingRad);
        }
        telemetry.update();
    }
}
