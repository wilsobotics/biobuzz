package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Match OpMode. The AutoTuner is wired in here: hold both bumpers and both triggers during INIT
 * and it tunes the very TeleOpDriver and Intake this OpMode will drive, so the match runs on the
 * measured values without anyone copying numbers around.
 *
 * Why this is a LinearOpMode: the tuner needs to drive the robot, ask questions on the gamepad and
 * be abortable by STOP. In an iterative OpMode a blocking call inside loop() cannot see STOP
 * (the SDK updates that flag on the loop thread), so the tuning run would be unstoppable. Inside
 * runOpMode() it runs on the OpMode thread with idle() doing the flag updates.
 *
 * The tuning run happens BEFORE waitForStart(), i.e. during INIT, so it costs no match time. That
 * also means the robot moves while the Driver Station says "init" - only start it with the robot
 * on the field and the area clear.
 */
@TeleOp(name = "Kindly rotate your wheels in accordance to my controller actions")
public class TeleOpController extends LinearOpMode {
    /** Which alliance this robot is on. Picks the start pose and the field frame from FieldMap. */
    private static final Alliance ALLIANCE = Alliance.RED;

    /**
     * Safety switch. The tuner drives the robot, so it only starts when all four of LB, RB, LT
     * and RT are held during INIT - nobody does that by accident. Set to false to make the
     * in-init tuning impossible; AutoTunerOpMode still tunes from its own OpMode.
     */
    private static final boolean ENABLE_INIT_TUNING = true;

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
    public void runOpMode() {
        driveTrain = new TeleOpDriver(hardwareMap, telemetry, gamepad1);
        intaker = new Intake(hardwareMap);
        driveTrain.initialiseDriver(ALLIANCE);

        if (ENABLE_INIT_TUNING && tuningRequested()) {
            runAutoTuner();
        } else {
            showInitHint();
        }

        waitForStart();

        while (opModeIsActive()) {
            driveLoop();
            idle();
        }
    }

    // ------------------------------------------------------------------
    // AutoTuner integration
    // ------------------------------------------------------------------

    /** LB + RB + LT + RT held together during INIT: deliberate, and impossible to hit by accident. */
    private boolean tuningRequested() {
        return gamepad1.left_bumper && gamepad1.right_bumper
                && gamepad1.left_trigger > SPEED_CAP_RELEASE_TRIGGER
                && gamepad1.right_trigger > SPEED_CAP_RELEASE_TRIGGER;
    }

    /**
     * Tunes the driver and intake this OpMode is holding, so the values are live for the match
     * that follows. keepRunning is "not stopped" rather than opModeIsActive(), because during
     * INIT the OpMode is not active yet.
     */
    private void runAutoTuner() {
        AutoTuner tuner = new AutoTuner(driveTrain, intaker, gamepad1, telemetry,
                () -> !isStopRequested(), this::yieldDuringInit);
        tuner.run();
        tuner.renderSummary();

        // The trials leave the robot somewhere else and re-zero its heading, so re-anchor the
        // field frame: wherever it stands now counts as its start tile.
        driveTrain.setPose(FieldMap.startPose(ALLIANCE));

        // Hand the robot back in the state the match expects.
        driveTrain.setHoldBrake(false);
        driveTrain.stopMotors();
        intaker.stopMotors();
        intaker.stop();

        telemetry.addLine("");
        telemetry.addLine("Tuned. Put the robot back on its start tile, then press START.");
        telemetry.update();
    }

    /**
     * Between tuning samples during INIT. idle() assumes the OpMode is running, so this yields
     * with a plain sleep instead; STOP still aborts because isStopRequested() is set by the SDK.
     */
    private void yieldDuringInit() {
        telemetry.update();
        try {
            Thread.sleep(2);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    private void showInitHint() {
        telemetry.addLine("Alliance: " + ALLIANCE
                + "   start pose: " + FieldMap.startPose(ALLIANCE));
        telemetry.addLine("Drive tuning: "
                + (driveTrain.isTuningLoadedFromStore() ? "saved values" : "source defaults"));
        telemetry.addLine("Intake tuning: "
                + (intaker.isSettingsLoadedFromStore() ? "saved values" : "source defaults"));
        if (ENABLE_INIT_TUNING) {
            telemetry.addLine("Hold LB + RB + LT + RT during INIT to AutoTune drive and intake.");
        }
        telemetry.update();
    }

    // ------------------------------------------------------------------
    // Match loop
    // ------------------------------------------------------------------

    private void driveLoop() {
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

        telemetry.update();
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
 * In INIT:
 * LB + RB + LT + RT held together - run the AutoTuner on this robot's real driver and intake
 *
 * */
