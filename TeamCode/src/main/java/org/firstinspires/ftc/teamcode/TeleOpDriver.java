package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Disabled
public class TeleOpDriver extends OpMode {
    GoBildaPinpointDriver odo;
    public DcMotor BackLeft;
    public DcMotor BackRight;
    public DcMotor FrontLeft;
    public DcMotor FrontRight;

    // Hold Break State (Approach A: Odometry-based Active Position Hold)
    public boolean isHoldBreakEnabled = false;
    public boolean isTargetPositionLocked = false;
    public Pose2D targetPose = null;

    // Tuning Parameters for Hold Break (Approach A)
    public double KP_TRANSLATIONAL = 0.0035; // Needs to be tuned - proportional gain for position error in MM
    public double KP_HEADING = 1.25;         // Needs to be tuned - proportional gain for heading error in Radians
    public double MAX_HOLD_POWER = 0.60;     // Needs to be tuned - maximum motor power cap for hold break correction
    public double JOYSTICK_DEADBAND = 0.05;  // Needs to be tuned - threshold for detecting driver joystick movement

    public void intialiseDriver() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        BackLeft = hardwareMap.get(DcMotor.class,"BackLeftMotor");
        BackRight= hardwareMap.get(DcMotor.class,"BackRightMotor");
        FrontLeft = hardwareMap.get(DcMotor.class,"FrontLeftMotor");
        FrontRight = hardwareMap.get(DcMotor.class,"FrontRightMotor");

        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Odometry Computer Config
        odo.setOffsets(-84.0,-168.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Odometry starting pos
        odo.resetPosAndIMU();
        Pose2D startingPosition = new Pose2D(DistanceUnit.MM,-923.925,1601.47, AngleUnit.RADIANS,0);
        odo.setPosition(startingPosition);

        // Telemetry
        telemetry.addData("Status", "Intialised");
        telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y Offset", odo.getYOffset(DistanceUnit.MM));
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();
    }

    @Override
    public void init() {
    }

    public void toggleHoldBreak() {
        isHoldBreakEnabled = !isHoldBreakEnabled;
        if (!isHoldBreakEnabled) {
            isTargetPositionLocked = false;
            targetPose = null;
        }
    }

    private double normalizeAngle(double radians) {
        double angle = radians;
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    public void GASGASGAS(double speedMultiplier) {
        double forward = 0.0;
        double strafe = 0.0;
        double rotate = 0.0;

        if (gamepad1 != null) {
            forward = -gamepad1.left_stick_y; // Inverted Y-axis
            strafe = gamepad1.left_stick_x;
            rotate = gamepad1.right_stick_x;
        }

        boolean isDriverInteracting = Math.abs(forward) > JOYSTICK_DEADBAND ||
                                      Math.abs(strafe) > JOYSTICK_DEADBAND ||
                                      Math.abs(rotate) > JOYSTICK_DEADBAND;

        Pose2D pos = odo.getPosition();
        double heading = pos.getHeading(AngleUnit.RADIANS);

        double globalForward;
        double globalStrafe;

        if (isHoldBreakEnabled && !isDriverInteracting) {
            // Joysticks neutral & Hold Break enabled -> active position hold
            if (!isTargetPositionLocked || targetPose == null) {
                targetPose = pos;
                isTargetPositionLocked = true;
            }

            // Calculate global field position errors
            double deltaX = targetPose.getX(DistanceUnit.MM) - pos.getX(DistanceUnit.MM);
            double deltaY = targetPose.getY(DistanceUnit.MM) - pos.getY(DistanceUnit.MM);
            double deltaHeading = normalizeAngle(targetPose.getHeading(AngleUnit.RADIANS) - heading);

            // Proportional field correction speeds
            globalForward = deltaX * KP_TRANSLATIONAL;
            globalStrafe = deltaY * KP_TRANSLATIONAL;
            rotate = deltaHeading * KP_HEADING;

            // Cap maximum power during hold correction
            globalForward = Math.max(-MAX_HOLD_POWER, Math.min(MAX_HOLD_POWER, globalForward));
            globalStrafe = Math.max(-MAX_HOLD_POWER, Math.min(MAX_HOLD_POWER, globalStrafe));
            rotate = Math.max(-MAX_HOLD_POWER, Math.min(MAX_HOLD_POWER, rotate));
        } else {
            // Driver is driving or Hold Break disabled -> unlock target pose
            isTargetPositionLocked = false;
            targetPose = null;

            double cosAngle = Math.cos(-heading);
            double sinAngle = Math.sin(-heading);

            globalStrafe = -forward * sinAngle + strafe * cosAngle;
            globalForward = forward * cosAngle + strafe * sinAngle;
        }

        double[] newWheelSpeeds = new double[4];

        newWheelSpeeds[0] = globalForward + globalStrafe + rotate;
        newWheelSpeeds[1] = globalForward - globalStrafe - rotate;
        newWheelSpeeds[2] = globalForward - globalStrafe + rotate;
        newWheelSpeeds[3] = globalForward + globalStrafe - rotate;

        double maxPower = 1.0;
        for (double wheelSpeed : newWheelSpeeds) {
            maxPower = Math.max(maxPower, Math.abs(wheelSpeed));
        }
        FrontLeft.setPower((newWheelSpeeds[0] / maxPower) * speedMultiplier);
        FrontRight.setPower((newWheelSpeeds[1] / maxPower) * speedMultiplier);
        BackLeft.setPower((newWheelSpeeds[2] / maxPower) * speedMultiplier);
        BackRight.setPower((newWheelSpeeds[3] / maxPower) * speedMultiplier);

        // More Telemetry
        telemetry.addData("Robot XPos: ", pos.getX(DistanceUnit.MM));
        telemetry.addData("Robot YPos: ", pos.getY(DistanceUnit.MM));
        telemetry.addData("Robot Heading: ", heading);
        telemetry.addData("Forward Speed: ", globalForward);
        telemetry.addData("Strafe Speed: ", globalStrafe);
        telemetry.addData("Hold Break Enabled: ", isHoldBreakEnabled);
        telemetry.addData("Target Pose Locked: ", isTargetPositionLocked);
        if (targetPose != null) {
            telemetry.addData("Target X: ", targetPose.getX(DistanceUnit.MM));
            telemetry.addData("Target Y: ", targetPose.getY(DistanceUnit.MM));
            telemetry.addData("Target Heading: ", targetPose.getHeading(AngleUnit.RADIANS));
        }
    }

    public void resetHeading() {
        odo.resetPosAndIMU();
    }

    public void DriveLoop(double speedMultiplier) {
        odo.update();
        GASGASGAS(speedMultiplier);

        telemetry.addData("RobotX: ", odo.getPosX(DistanceUnit.MM));
        telemetry.addData("RobotY: ", odo.getPosY(DistanceUnit.MM));
        telemetry.addData("Robot Heading: ", odo.getHeading(AngleUnit.RADIANS));
        telemetry.update();
    }

    @Override
    public void loop() {
    }
}
