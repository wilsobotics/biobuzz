package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Measures the tuning values used by TeleOpDriver, TeleOpController and Intake, then prints a
 * block of numbers to paste back into the code. Nothing is written to flash - the values live in
 * the driver's Tuning object and the intake's Settings object for the length of this run only.
 *
 * WHAT IT ACTUALLY MEASURES (no human judgement needed):
 *   - joystick deadband       from the sticks' own noise floor
 *   - MIN_HOLD_POWER          the power at which the robot first breaks static friction
 *   - KP_TRANSLATIONAL        sweep of candidate gains, scored on overshoot + settling error
 *   - KP_HEADING              same, for rotation
 *   - hold deadbands          from the position/heading noise while holding still
 *   - beam-break polarity     from the sensor's clear vs blocked levels
 *   - minimum roller power    the power at which each roller encoder first reads motion
 *   - roller direction        confirmed by you watching the balls, then applied
 *
 * WHAT IT CANNOT MEASURE (a human decides):
 *   - pod offsets             a tape measure, not a sensor (verifies the result only)
 *   - starting pose           depends where you place the robot
 *   - MAX_HOLD_POWER          a safety ceiling, not a physical property
 *   - intake powers           depend on how your robot feeds and what "jams" means
 *   - door open/closed        no feedback on the servo's actual position
 *   - MAX_BALLS, cooldown, eject time   game strategy, not physics
 *
 * The robot drives itself during phases 3-6, so give it about 1 m of clear floor in front and
 * keep hands off the controller until it asks. Press STOP at any time to abort.
 */
@TeleOp(name = "Auto Tuner", group = "Tuning")
public class AutoTuner extends LinearOpMode {

    // ---- how hard the tests push ----
    private static final double PUSH_POWER = 0.45;
    private static final double PUSH_TARGET_MM = 150.0;
    private static final double PUSH_TIMEOUT_SEC = 2.0;
    private static final double HOLD_SETTLE_SECONDS = 2.5;
    private static final double HEADING_PUSH_POWER = 0.5;
    private static final double HEADING_PUSH_RAD = Math.toRadians(25);
    private static final double MOTION_THRESHOLD_MM = 3.0;
    private static final double SETTLED_ERROR_MM = 30.0;
    private static final double SETTLED_ERROR_DEG = 3.0;
    private static final double ROLLER_MOTION_TICKS_PER_SEC = 20.0;

    private static final double[] TRANSLATIONAL_CANDIDATES =
            {0.0015, 0.0025, 0.0035, 0.0050, 0.0070, 0.0100};
    private static final double[] HEADING_CANDIDATES =
            {0.5, 0.8, 1.25, 1.8, 2.5};

    private TeleOpDriver driveTrain;
    private TeleOpDriver.Tuning tuning;
    private Intake intake;
    private Intake.Settings intakeSettings;

    private boolean driveReady = false;
    private boolean intakeReady = false;
    private String driveError = "";
    private String intakeError = "";

    private boolean odometryOk = false;
    private String odometryNote = "not checked";
    private String beamNote = "not checked";
    private String directionNote = "not checked";
    private double lowerMinSpin = -1;
    private double upperMinSpin = -1;

    /** Overshoot and resting error from one hold trial. */
    private static class TrialResult {
        final double finalError;
        final double overshoot;
        final boolean settled;

        TrialResult(double finalError, double overshoot, boolean settled) {
            this.finalError = finalError;
            this.overshoot = overshoot;
            this.settled = settled;
        }

        double score() {
            return finalError + 2.0 * overshoot + (settled ? 0 : 1000.0);
        }
    }

    @Override
    public void runOpMode() {
        tuning = new TeleOpDriver.Tuning();
        intakeSettings = new Intake.Settings();

        render("AUTO TUNER",
                "",
                "The robot will drive itself. Put it on the field with",
                "about 1 m of clear floor ahead, hands off the sticks.",
                "",
                "It measures the hold-brake gains, deadbands, the",
                "friction floor and the intake sensors, then prints",
                "every value to copy back into the code.",
                "",
                "Press START to begin. STOP aborts at any point.");

        driveTrain = new TeleOpDriver(hardwareMap, telemetry, gamepad1, tuning);
        driveTrain.setTelemetryEnabled(false);
        try {
            driveTrain.initialiseDriver();
            driveReady = true;
        } catch (RuntimeException e) {
            driveError = e.getClass().getSimpleName() + ": " + e.getMessage();
        }

        try {
            intake = new Intake(hardwareMap, intakeSettings);
            intake.stopMotors();
            intakeReady = true;
        } catch (RuntimeException e) {
            intakeError = e.getClass().getSimpleName() + ": " + e.getMessage();
        }

        waitForStart();
        if (!opModeIsActive()) {
            return;
        }

        if (driveReady) {
            odometryOk = checkOdometry();
            measureStickDeadband();
            if (odometryOk) {
                measureFrictionFloor();
                tuneTranslationalGain();
                measureHoldDeadbands();
                tuneHeadingGain();
            }
            driveTrain.stopMotors();
            driveTrain.setHoldBrake(false);
        }

        if (intakeReady) {
            tuneIntake();
        }

        // Leave the robot safe and show the results until the operator stops the OpMode.
        if (driveReady) {
            driveTrain.setHoldBrake(false);
            driveTrain.stopMotors();
        }
        if (intakeReady) {
            intake.stopMotors();
        }
        showSummaryUntilStopped();
    }

    // =================================================================
    // PHASE 1 - odometry sanity, and pod direction discovery
    // =================================================================
    private boolean checkOdometry() {
        if (!driveTrain.isOdometryReady()) {
            odometryNote = "Pinpoint not READY (" + driveTrain.getOdometryStatus() + ")";
            askYesNo("PHASE 1: ODOMETRY",
                    "The Pinpoint is not READY, so drive tuning is skipped.",
                    "Press A to continue with the intake phases.");
            return false;
        }

        driveTrain.setPodDirections(true, true);
        driveTrain.resetPoseToOrigin();

        // Drive forward and ask the operator whether the robot really went forward. Without that
        // answer a negative X reading could mean "pod backwards" or "motors backwards", and
        // flipping the wrong one hides a wiring fault.
        driveTrain.driveOpenLoop(0.5, 0, 0);
        waitSeconds(0.7);
        driveTrain.stopMotors();
        waitSeconds(0.3);
        double forwardDelta = driveTrain.readPose().getX(DistanceUnit.MM);

        boolean wentForward = askYesNo("PHASE 1: ODOMETRY",
                "The robot just drove forward at 0.5 power for 0.7 s.",
                "Did it actually move FORWARD?  A = yes   B = no");

        if (!wentForward) {
            odometryNote = "robot did not move forward: fix motor directions in initialiseDriver()";
            return false;
        }

        if (forwardDelta < 0) {
            driveTrain.setPodDirections(false, true);
            odometryNote = "X pod flipped to REVERSE (X fell while driving forward)";
        } else {
            odometryNote = "X pod direction correct";
        }

        // Strafe right: the strafe pod should read -Y, because +Y is left.
        driveTrain.resetPoseToOrigin();
        driveTrain.driveOpenLoop(0, 0.5, 0);
        waitSeconds(0.7);
        driveTrain.stopMotors();
        waitSeconds(0.3);
        double strafeDelta = driveTrain.readPose().getY(DistanceUnit.MM);

        boolean wentRight = askYesNo("PHASE 1: ODOMETRY",
                "The robot just strafed RIGHT at 0.5 power for 0.7 s.",
                "Did it actually strafe to its RIGHT?  A = yes   B = no");

        if (wentRight && strafeDelta > 0) {
            driveTrain.setPodDirections(driveTrain.isXPodForward(), false);
            odometryNote += "; Y pod flipped to REVERSE";
        }

        // Verify the X fix actually took.
        driveTrain.resetPoseToOrigin();
        driveTrain.driveOpenLoop(0.45, 0, 0);
        waitSeconds(0.6);
        driveTrain.stopMotors();
        waitSeconds(0.2);
        if (driveTrain.readPose().getX(DistanceUnit.MM) <= 0) {
            odometryNote += "; X still not increasing - check the pod cable and offset";
            return false;
        }

        driveTrain.resetPoseToOrigin();
        driveTrain.resetHeading();
        return true;
    }

    // =================================================================
    // PHASE 2 - joystick deadband from the sticks' own noise
    // =================================================================
    private void measureStickDeadband() {
        render("PHASE 2: JOYSTICK DEADBAND",
                "",
                "Hands off both sticks for 2 seconds.");

        waitSeconds(1.0);
        double noise = 0;
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < 2.0) {
            noise = Math.max(noise, Math.abs(gamepad1.left_stick_y));
            noise = Math.max(noise, Math.abs(gamepad1.left_stick_x));
            noise = Math.max(noise, Math.abs(gamepad1.right_stick_x));
            idle();
        }

        double engage = Math.max(0.05, noise + 0.03);
        double release = Math.max(0.03, Math.min(engage - 0.01, noise + 0.015));
        tuning.joystickEngageDeadband = engage;
        tuning.joystickReleaseDeadband = release;

        render("PHASE 2: JOYSTICK DEADBAND",
                String.format("stick noise      %.3f", noise),
                String.format("engage deadband  %.3f", engage),
                String.format("release deadband %.3f", release));
        waitSeconds(1.0);
    }

    // =================================================================
    // PHASE 3 - the power at which the robot starts to move
    // =================================================================
    private void measureFrictionFloor() {
        render("PHASE 3: FRICTION FLOOR",
                "",
                "Ramping wheel power until the robot moves.",
                "Keep clear of the robot.");

        driveTrain.setHoldBrake(false);
        driveTrain.resetPoseToOrigin();

        double found = 0;
        for (double power = 0.02; power <= 0.201; power += 0.02) {
            if (!opModeIsActive()) return;

            driveTrain.driveOpenLoop(power, 0, 0);
            waitSeconds(0.45);
            driveTrain.stopMotors();
            waitSeconds(0.25);

            double moved = Math.abs(driveTrain.readPose().getX(DistanceUnit.MM));
            if (moved >= MOTION_THRESHOLD_MM) {
                found = power;
                break;
            }
        }

        driveTrain.resetPoseToOrigin();
        tuning.minHoldPower = found > 0 ? Math.min(0.15, found + 0.01) : 0.06;

        render("PHASE 3: FRICTION FLOOR",
                found > 0 ? String.format("first movement at %.2f power", found)
                          : "no movement measured - keeping default",
                String.format("MIN_HOLD_POWER = %.3f", tuning.minHoldPower));
        waitSeconds(1.2);
    }

    // =================================================================
    // PHASE 4 - translational gain sweep
    // =================================================================
    private void tuneTranslationalGain() {
        double bestGain = tuning.kpTranslational;
        double bestScore = Double.MAX_VALUE;

        for (double candidate : TRANSLATIONAL_CANDIDATES) {
            if (!opModeIsActive()) return;

            tuning.kpTranslational = candidate;
            TrialResult result = runTranslationTrial();

            if (result.score() < bestScore) {
                bestScore = result.score();
                bestGain = candidate;
            }

            render("PHASE 4: KP_TRANSLATIONAL",
                    String.format("candidate %.4f", candidate),
                    String.format("rest error %.1f mm", result.finalError),
                    String.format("overshoot  %.1f mm", result.overshoot),
                    result.settled ? "settled" : "did not settle",
                    "",
                    String.format("best so far %.4f", bestGain));
        }

        tuning.kpTranslational = bestGain;

        render("PHASE 4: KP_TRANSLATIONAL",
                String.format("chosen KP_TRANSLATIONAL = %.4f", bestGain));
        waitSeconds(1.2);
    }

    /** Pushes the robot off target, then lets the hold drive it back and scores the recovery. */
    private TrialResult runTranslationTrial() {
        driveTrain.setHoldBrake(false);
        driveTrain.resetHeading();
        driveTrain.resetPoseToOrigin();

        ElapsedTime pushTimer = new ElapsedTime();
        while (opModeIsActive() && pushTimer.seconds() < PUSH_TIMEOUT_SEC) {
            driveTrain.driveOpenLoop(PUSH_POWER, 0, 0);
            if (driveTrain.readPose().getX(DistanceUnit.MM) >= PUSH_TARGET_MM) break;
            idle();
        }
        driveTrain.stopMotors();
        waitSeconds(0.4);

        // Target is where the trial started; the hold has to bring it back from +150 mm.
        driveTrain.setHoldBrake(true);
        driveTrain.setHoldTarget(0, 0, 0);

        double overshoot = 0;
        double finalError = Double.MAX_VALUE;
        ElapsedTime settleTimer = new ElapsedTime();
        while (opModeIsActive() && settleTimer.seconds() < HOLD_SETTLE_SECONDS) {
            driveTrain.drive(1.0);
            double x = driveTrain.getPose().getX(DistanceUnit.MM);
            if (-x > overshoot) overshoot = -x;
            finalError = Math.abs(x);
            idle();
        }

        driveTrain.setHoldBrake(false);
        driveTrain.stopMotors();
        return new TrialResult(finalError, overshoot, finalError <= SETTLED_ERROR_MM);
    }

    // =================================================================
    // PHASE 5 - deadbands from the noise while holding still
    // =================================================================
    private void measureHoldDeadbands() {
        driveTrain.setHoldBrake(false);
        driveTrain.resetPoseToOrigin();
        driveTrain.resetHeading();
        driveTrain.setHoldBrake(true);
        driveTrain.setHoldTarget(0, 0, 0);
        waitSeconds(0.8);

        double maxX = 0;
        double maxY = 0;
        double maxHeading = 0;
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < 2.0) {
            driveTrain.drive(1.0);
            Pose2D pos = driveTrain.getPose();
            maxX = Math.max(maxX, Math.abs(pos.getX(DistanceUnit.MM)));
            maxY = Math.max(maxY, Math.abs(pos.getY(DistanceUnit.MM)));
            maxHeading = Math.max(maxHeading, Math.abs(pos.getHeading(AngleUnit.RADIANS)));
            idle();
        }

        driveTrain.setHoldBrake(false);
        driveTrain.stopMotors();

        double translationNoise = Math.max(maxX, maxY);
        tuning.holdPositionDeadbandMm =
                clamp(translationNoise + 3.0, 5.0, 30.0);
        tuning.holdHeadingDeadbandRad =
                clamp(maxHeading + Math.toRadians(0.5), Math.toRadians(1.0), Math.toRadians(6.0));

        render("PHASE 5: HOLD DEADBANDS",
                String.format("position noise %.1f mm", translationNoise),
                String.format("heading noise  %.2f deg", Math.toDegrees(maxHeading)),
                String.format("position deadband %.1f mm", tuning.holdPositionDeadbandMm),
                String.format("heading deadband  %.2f deg",
                        Math.toDegrees(tuning.holdHeadingDeadbandRad)));
        waitSeconds(1.2);
    }

    // =================================================================
    // PHASE 6 - heading gain sweep
    // =================================================================
    private void tuneHeadingGain() {
        double bestGain = tuning.kpHeading;
        double bestScore = Double.MAX_VALUE;

        for (double candidate : HEADING_CANDIDATES) {
            if (!opModeIsActive()) return;

            tuning.kpHeading = candidate;
            TrialResult result = runHeadingTrial();

            if (result.score() < bestScore) {
                bestScore = result.score();
                bestGain = candidate;
            }

            render("PHASE 6: KP_HEADING",
                    String.format("candidate %.2f", candidate),
                    String.format("rest error %.2f deg", result.finalError),
                    String.format("overshoot  %.2f deg", result.overshoot),
                    result.settled ? "settled" : "did not settle",
                    "",
                    String.format("best so far %.2f", bestGain));
        }

        tuning.kpHeading = bestGain;

        render("PHASE 6: KP_HEADING",
                String.format("chosen KP_HEADING = %.2f", bestGain));
        waitSeconds(1.2);
    }

    /** Spins the robot off heading, then holds position while the heading is driven back to 0. */
    private TrialResult runHeadingTrial() {
        driveTrain.setHoldBrake(false);
        driveTrain.resetHeading();

        ElapsedTime pushTimer = new ElapsedTime();
        while (opModeIsActive() && pushTimer.seconds() < PUSH_TIMEOUT_SEC) {
            driveTrain.driveOpenLoop(0, 0, HEADING_PUSH_POWER);
            if (Math.abs(driveTrain.readPose().getHeading(AngleUnit.RADIANS)) >= HEADING_PUSH_RAD) break;
            idle();
        }
        driveTrain.stopMotors();
        waitSeconds(0.4);

        // Hold the current position, but the original heading - so only rotation is corrected.
        Pose2D start = driveTrain.readPose();
        driveTrain.setHoldBrake(true);
        driveTrain.setHoldTarget(start.getX(DistanceUnit.MM), start.getY(DistanceUnit.MM), 0);

        double overshoot = 0;
        double finalError = Double.MAX_VALUE;
        ElapsedTime settleTimer = new ElapsedTime();
        while (opModeIsActive() && settleTimer.seconds() < HOLD_SETTLE_SECONDS) {
            driveTrain.drive(1.0);
            double heading = driveTrain.getPose().getHeading(AngleUnit.RADIANS);
            if (-heading > overshoot) overshoot = -heading;
            finalError = Math.abs(heading);
            idle();
        }

        driveTrain.setHoldBrake(false);
        driveTrain.stopMotors();

        return new TrialResult(Math.toDegrees(finalError), Math.toDegrees(overshoot),
                Math.toDegrees(finalError) <= SETTLED_ERROR_DEG);
    }

    // =================================================================
    // PHASE 7 - intake sensors, roller direction and roller floor
    // =================================================================
    private void tuneIntake() {
        waitForA("PHASE 7: INTAKE",
                "Make sure the beam-break is CLEAR (no ball in it),",
                "then press A.");

        boolean clearRaw = intake.getRawBeamState();

        while (opModeIsActive()) {
            render("PHASE 7: INTAKE",
                    "Now BLOCK the beam with a ball and hold it there.",
                    "Press B when it is blocked.");
            if (gamepad1.b) {
                waitButtonRelease();
                break;
            }
            idle();
        }

        boolean blockedRaw = intake.getRawBeamState();

        if (clearRaw == blockedRaw) {
            beamNote = "sensor did not change between clear and blocked - polarity left alone";
        } else {
            intakeSettings.beamActiveLow = !blockedRaw;
            beamNote = "blocked reads " + (blockedRaw ? "HIGH" : "LOW")
                    + " -> beamActiveLow = " + intakeSettings.beamActiveLow;
        }

        lowerMinSpin = findMinSpinPower(true);
        upperMinSpin = findMinSpinPower(false);

        // Direction confirmation: run the intake and watch which way the balls go.
        runIntakeFor(2.5);
        boolean inward = askYesNo("PHASE 7: INTAKE",
                "The rollers just ran INTAKE for 2.5 s.",
                "Did they pull the balls INWARD?  A = yes   B = no");

        if (!inward) {
            intakeSettings.lowerReversed = !intakeSettings.lowerReversed;
            intakeSettings.upperReversed = !intakeSettings.upperReversed;
            intake.applyDirections();
            runIntakeFor(2.5);
            directionNote = "both roller directions flipped in Settings";
        } else {
            directionNote = "roller directions left as they were";
        }

        intake.stopMotors();
        intake.stop();

        render("PHASE 7: INTAKE",
                beamNote,
                String.format("lower min spin %s", formatSpin(lowerMinSpin)),
                String.format("upper min spin %s", formatSpin(upperMinSpin)),
                directionNote);
        waitSeconds(1.2);
    }

    private void runIntakeFor(double seconds) {
        intake.intake();
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < seconds) {
            intake.periodic();
            idle();
        }
        intake.stopMotors();
    }

    /** Ramps one roller until its encoder reports motion; -1 if no encoder is readable. */
    private double findMinSpinPower(boolean lower) {
        for (double power = 0.05; power <= 0.601; power += 0.05) {
            if (!opModeIsActive()) return -1;

            intake.setTestPower(lower ? power : 0, lower ? 0 : power);
            waitSeconds(0.5);

            double velocity = Math.abs(lower ? intake.getLowerVelocity() : intake.getUpperVelocity());
            if (velocity > ROLLER_MOTION_TICKS_PER_SEC) {
                intake.stopMotors();
                return power;
            }
        }
        intake.stopMotors();
        return -1; // encoder missing: the operator has to judge this one by eye
    }

    private String formatSpin(double power) {
        return power < 0 ? "unreadable (no encoder?)" : String.format("%.2f power", power);
    }

    // =================================================================
    // SUMMARY
    // =================================================================
    private void showSummaryUntilStopped() {
        while (opModeIsActive()) {
            telemetry.addLine("=== RESULTS: copy into the code ===");
            telemetry.addLine("");

            if (driveReady) {
                telemetry.addLine("TeleOpDriver.Tuning defaults:");
                telemetry.addData("kpTranslational", "%.4f", tuning.kpTranslational);
                telemetry.addData("kpHeading", "%.2f", tuning.kpHeading);
                telemetry.addData("minHoldPower", "%.3f", tuning.minHoldPower);
                telemetry.addData("holdPositionDeadbandMm", "%.1f", tuning.holdPositionDeadbandMm);
                telemetry.addData("holdHeadingDeadbandRad", "%.4f (%.2f deg)",
                        tuning.holdHeadingDeadbandRad,
                        Math.toDegrees(tuning.holdHeadingDeadbandRad));
                telemetry.addData("joystickEngageDeadband", "%.3f", tuning.joystickEngageDeadband);
                telemetry.addData("joystickReleaseDeadband", "%.3f", tuning.joystickReleaseDeadband);
                telemetry.addData("maxHoldPower", "%.2f (unchanged, safety cap)", tuning.maxHoldPower);
                telemetry.addLine("Pods: X " + (driveTrain.isXPodForward() ? "FORWARD" : "REVERSE")
                        + ", Y " + (driveTrain.isYPodForward() ? "FORWARD" : "REVERSE"));
                telemetry.addData("Odometry", driveTrain.getOdometryStatus());
                telemetry.addLine("odometry note: " + odometryNote);
                telemetry.addLine("");
            } else {
                telemetry.addLine("Driver unavailable: " + driveError);
                telemetry.addLine("");
            }

            if (intakeReady) {
                telemetry.addLine("Intake.Settings defaults (only measured ones shown):");
                telemetry.addData("beamActiveLow", "%b", intakeSettings.beamActiveLow);
                telemetry.addData("lowerReversed", "%b", intakeSettings.lowerReversed);
                telemetry.addData("upperReversed", "%b", intakeSettings.upperReversed);
                telemetry.addData("beam", intake.getSensorDebug());
                telemetry.addData("min spin", "lower %s / upper %s",
                        formatSpin(lowerMinSpin), formatSpin(upperMinSpin));
                telemetry.addLine("beam note: " + beamNote);
                telemetry.addLine("direction: " + directionNote);
                telemetry.addLine("Still yours to set: intakePower, fullPower,");
                telemetry.addLine("reversePower, ejectPower, shootPower, doorOpen,");
                telemetry.addLine("doorClosed, maxBalls, countCooldownSeconds,");
                telemetry.addLine("ejectTimeSeconds.");
            } else {
                telemetry.addLine("Intake unavailable: " + intakeError);
            }

            telemetry.addLine("");
            telemetry.addLine("Press STOP when you have written them down.");
            telemetry.update();
        }
    }

    // =================================================================
    // Small helpers
    // =================================================================
    private void render(String... lines) {
        for (String line : lines) {
            telemetry.addLine(line);
        }
        telemetry.update();
    }

    private boolean askYesNo(String title, String line1, String line2) {
        while (opModeIsActive()) {
            render(title, "", line1, "", line2);
            if (gamepad1.a) {
                waitButtonRelease();
                return true;
            }
            if (gamepad1.b) {
                waitButtonRelease();
                return false;
            }
            idle();
        }
        return false;
    }

    private void waitForA(String title, String line1, String line2) {
        while (opModeIsActive()) {
            render(title, "", line1, line2);
            if (gamepad1.a) {
                waitButtonRelease();
                return;
            }
            idle();
        }
    }

    private void waitButtonRelease() {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && (gamepad1.a || gamepad1.b) && timer.seconds() < 2.0) {
            idle();
        }
    }

    private void waitSeconds(double seconds) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < seconds) {
            idle();
        }
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
