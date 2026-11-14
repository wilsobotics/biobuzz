package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Kindly rotate your wheels in accordance to my controller actions")
public class TeleOpController extends OpMode {
    private static final double SPEED_CAP_NORMAL = 0.75;
    private static final double SPEED_CAP_FULL = 1.0;
    private static final double SPEED_CAP_RELEASE_TRIGGER = 0.5;

    private TeleOpDriver driveTrain;
    private Intake intaker;

    private boolean lastY = false;
    private boolean lastL1 = false;
    private boolean lastRightBumper = false;
    private boolean intakeOn = false;

    @Override
    public void init() {
        driveTrain = new TeleOpDriver(hardwareMap, telemetry, gamepad1);
        intaker = new Intake(hardwareMap);
        driveTrain.initialiseDriver();
    }

    @Override
    public void loop() {
        // Default cap of 0.75 power, removed (1.0) while the LEFT trigger (L2) is held past halfway
        double speedCap = SPEED_CAP_NORMAL;
        if (gamepad1.left_trigger > SPEED_CAP_RELEASE_TRIGGER) {
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
        if (gamepad1.right_trigger > SPEED_CAP_RELEASE_TRIGGER) {
            intaker.shoot();
        } else if (intakeOn) {
            intaker.intake();
        } else {
            intaker.stop();
        }
        // The intake only moves hardware inside periodic(), so it has to run every loop.
        intaker.periodic();

        // Run drive loop with the dynamic speed multiplier
        driveTrain.drive(speedCap);

        telemetry.addData("Intake State: ", intaker.getState());
        telemetry.addData("Balls: ", intaker.getBallCount());
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
