package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Kindly rotate your wheels in accordance to my controller actions")
public class TeleOpController extends OpMode {
    private static final double SPEED_CAP_NORMAL = 0.75;
    private static final double SPEED_CAP_FULL = 1.0;
    /** Left trigger past these releases the 0.75 precision cap. */
    private static final double SPEED_CAP_TRIGGER = 0.5;
    /** Right trigger past these counts as "shoot". Same number today, a different job. */
    private static final double SHOOT_TRIGGER = 0.5;

    private TeleOpDriver driveTrain;
    private Intake intaker;

    private boolean lastY = false;
    private boolean lastL1 = false;
    private boolean lastRightBumper = false;
    private boolean intakeOn = false;

    private boolean driveReady = false;
    private String driveFault = "";

    @Override
    public void init() {
        // Telemetry goes to the Driver Station and the Slothboard at once. The driver gets the
        // wrapped object, so its own addData calls reach the dashboard too.
        telemetry = new MultipleTelemetry(telemetry,
                FtcDashboard.getInstance().getTelemetry());

        driveTrain = new TeleOpDriver(hardwareMap, telemetry, gamepad1);
        intaker = new Intake(hardwareMap);
        try {
            driveTrain.initialiseDriver();
            driveReady = true;
        } catch (RuntimeException e) {
            driveFault = e.getClass().getSimpleName() + ": " + e.getMessage();
        }
        if (!driveReady) telemetry.addLine("DRIVE UNAVAILABLE - " + driveFault);
        if (!intaker.isAvailable()) telemetry.addLine("Intake unavailable - " + intaker.getFault());
    }


    @Override
    public void loop() {
        // Default cap of 0.75 power, removed (1.0) while the LEFT trigger (L2) is held past halfway
        double speedCap = SPEED_CAP_NORMAL;
        if (gamepad1.left_trigger > SPEED_CAP_TRIGGER) {
            speedCap = SPEED_CAP_FULL;
        }

        // Reset heading check
        boolean currentY = gamepad1.y;
        if (currentY && !lastY) {
            driveTrain.resetHeading();
        }
        lastY = currentY;

        // Hold Brake toggle check (L1 / left_bumper)
        boolean currentL1 = gamepad1.left_bumper;
        if (currentL1 && !lastL1) {
            driveTrain.toggleHoldBrake();
        }
        lastL1 = currentL1;

        // Intake toggle (R1). Edge-detected, otherwise it flips every loop while held.
        boolean currentRightBumper = gamepad1.right_bumper;
        if (currentRightBumper && !lastRightBumper) {
            intakeOn = !intakeOn;
        }
        lastRightBumper = currentRightBumper;

        // R2 shoots while held, otherwise run or stop the intake
        if (gamepad1.right_trigger > SHOOT_TRIGGER) {
            intaker.shoot();
        } else if (intakeOn) {
            intaker.intake();
        } else {
            intaker.stop();
        }

        if (gamepad1.b) {
            intaker.reverse();
        }
        // The intake only moves hardware inside periodic(), so it has to run every loop.
        intaker.periodic();

        // Reported BEFORE drive(): drive() ends with telemetry.update(), so anything added after
        // it is held back and lands one frame late.
        if (intaker.isAvailable()) {
            telemetry.addData("Intake State: ", intaker.getState());
            // The count is only trustworthy between a shot and the next ejected extra ball.
            telemetry.addData("Balls (since last shot): ", intaker.getBallCount());
            telemetry.addData("Beam: ", intaker.getSensorDebug());
        } else {
            telemetry.addData("Intake: ", "unavailable - " + intaker.getFault());
        }

        // Run drive loop with the dynamic speed multiplier
        if (driveReady) driveTrain.drive(speedCap);
    }

    @Override
    public void stop() {
        if (driveReady) {
            driveTrain.setHoldBrake(false); // also clears the target
            driveTrain.stopMotors();
        }
        intaker.stopMotors(); // immediate, not stop()
    }

}

/*
 * Controls:
 * Y button - zero Field-centric drive heading (position is kept)
 * L2 Held more than half-way - removes the 0.75 "precision" speed cap and allows you to reach full speed
 * R2 Held - shoot (opens the door and feeds the flywheel)
 * R1 - Toggle the intake on and off
 * Left Joystick Y - Forwards and backwards movement
 * Left Joystick X - Left and Right Strafing
 * Right Joystick X - Rotation Clockwise and Counter-Clockwise
 * L1 - Hold Brake Toggle (Active Position Hold via Odometry); the right stick still turns while held
 *
 * */
