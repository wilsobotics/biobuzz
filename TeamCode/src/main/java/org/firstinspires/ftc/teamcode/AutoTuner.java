package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.function.BooleanSupplier;

/**
 * Measures the values that TeleOpDriver and Intake run on and writes them straight into the live
 * {@link TeleOpDriver.Tuning} and {@link Intake.Settings} objects handed to the constructor.
 *
 * There is no summary to copy back into the source: the driver and intake passed in are the ones
 * being tuned, so the next drive() / periodic() call already uses the measured numbers. In
 * TeleOpController the tuner runs before the match starts (see ENABLE_INIT_TUNING), so the match
 * drives on the values it just measured; AutoTunerOpMode runs it on its own for bench sessions.
 *
 * <h2>What it measures (no human judgement needed)</h2>
 * <ul>
 *   <li>joystick deadband - from the sticks' own noise floor</li>
 *   <li>Tuning.minHoldPower - the power at which the robot first breaks static friction</li>
 *   <li>Tuning.kpTranslational - sweep of candidate gains, scored on overshoot + settling error</li>
 *   <li>Tuning.kpHeading - same, for rotation</li>
 *   <li>hold deadbands - from the position/heading noise while holding still</li>
 *   <li>pod directions - from which way the odometry actually moves</li>
 *   <li>beam-break polarity - from the sensor's clear vs blocked levels</li>
 *   <li>Intake.Settings min roller power and roller direction (direction confirmed by you)</li>
 * </ul>
 *
 * <h2>What it cannot measure (a human decides)</h2>
 * <ul>
 *   <li>pod offsets - a tape measure, not a sensor (the run verifies them only)</li>
 *   <li>starting pose - depends where you place the robot (that lives in FieldMap)</li>
 *   <li>maxHoldPower - a safety ceiling, not a physical property</li>
 *   <li>intake powers - depend on how your robot feeds and what "jams" means</li>
 *   <li>door open/closed - no feedback on the servo's actual position</li>
 *   <li>maxBalls, cooldown, eject time - game strategy, not physics</li>
 * </ul>
 *
 * <h2>Safety</h2>
 * The robot drives itself during phase 3-6, so give it about 1 m of clear floor and keep hands
 * off the controller until it asks. Everything runs through the caller's keepRunning/tick pair,
 * so STOP aborts at any point and the phases always leave the robot with the motors at zero and
 * the hold brake off. Nothing is written to flash: a new OpMode run starts from the coded
 * defaults, and the results are saved on the Control Hub (TuneStore) at the end of the run, so
 * the next OpMode loads them instead of tuning again.
 */
public final class AutoTuner {

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

    private final TeleOpDriver driver;
    private final Intake intake;                 // may be null: intake phases are then skipped
    private final Gamepad gamepad;
    private final Telemetry telemetry;
    private final BooleanSupplier keepRunning;
    private final Runnable yieldToSdk;
    private final TeleOpDriver.Tuning tuning;
    private final Intake.Settings intakeSettings;

    private boolean driveTuned = false;
    private boolean intakeTuned = false;
    private boolean driveSaved = false;
    private boolean intakeSaved = false;
    private String odometryNote = "not checked";
    private String beamNote = "not checked";
    private String directionNote = "not checked";
    private double lowerMinSpin = -1;
    private double upperMinSpin = -1;

    /**
     * @param driver      the drive train to tune; its Tuning object is edited in place
     * @param intake      the intake to tune, or null to skip the intake phases
     * @param gamepad     the pad used for the yes/no prompts
     * @param telemetry   where the running commentary goes
     * @param keepRunning false once STOP is pressed; every phase checks it
     * @param yieldToSdk  what to do between samples (LinearOpMode.idle())
     */
    public AutoTuner(TeleOpDriver driver,
                     Intake intake,
                     Gamepad gamepad,
                     Telemetry telemetry,
                     BooleanSupplier keepRunning,
                     Runnable yieldToSdk) {
        this.driver = driver;
        this.intake = intake;
        this.gamepad = gamepad;
        this.telemetry = telemetry;
        this.keepRunning = keepRunning;
        this.yieldToSdk = yieldToSdk;
        this.tuning = driver.getTuning();
        this.intakeSettings = intake == null ? null : intake.getSettings();
    }

    /** Runs every phase it can, writing results into the live Tuning / Settings objects. */
    public void run() {
        driveTuned = checkOdometry();
        measureStickDeadband();

        if (driveTuned) {
            measureFrictionFloor();
            tuneTranslationalGain();
            measureHoldDeadbands();
            tuneHeadingGain();
        }
        driver.setHoldBrake(false);
        driver.stopMotors();

        if (intake != null) {
            tuneIntake();
            intakeTuned = true;
            intake.stopMotors();
            intake.stop();
        }

        // Only save what was actually measured: a skipped phase leaves the previous save intact.
        if (driveTuned) {
            driveSaved = driver.saveTuning();
        }
        if (intakeTuned) {
            intakeSaved = intake.saveSettings();
        }

        driver.setHoldBrake(false);
        driver.stopMotors();
    }

    // =================================================================
    // PHASE 1 - odometry sanity, and pod direction discovery
    // =================================================================
    private boolean checkOdometry() {
        if (!driver.isOdometryReady()) {
            odometryNote = "Pinpoint not READY (" + driver.getOdometryStatus() + ")";
            askYesNo("PHASE 1: ODOMETRY",
                    "The Pinpoint is not READY, so drive tuning is skipped.",
                    "Press A to continue with the intake phases.");
            return false;
        }

        driver.setPodDirections(true, true);
        driver.resetPoseToOrigin();

        // Drive forward and ask the operator whether the robot really went forward. Without that
        // answer a negative X reading could mean "pod backwards" or "motors backwards", and
        // flipping the wrong one hides a wiring fault.
        driver.driveOpenLoop(0.5, 0, 0);
        waitSeconds(0.7);
        driver.stopMotors();
        waitSeconds(0.3);
        double forwardDelta = driver.readPose().getX(DistanceUnit.MM);

        boolean wentForward = askYesNo("PHASE 1: ODOMETRY",
                "The robot just drove forward at 0.5 power for 0.7 s.",
                "Did it actually move FORWARD?  A = yes   B = no");

        if (!wentForward) {
            odometryNote = "robot did not move forward: fix motor directions in initialiseDriver()";
            return false;
        }

        if (forwardDelta < 0) {
            driver.setPodDirections(false, true);
            odometryNote = "X pod flipped to REVERSE (X fell while driving forward)";
        } else {
            odometryNote = "X pod direction correct";
        }

        // Strafe right: the strafe pod should read -Y, because +Y is left.
        driver.resetPoseToOrigin();
        driver.driveOpenLoop(0, 0.5, 0);
        waitSeconds(0.7);
        driver.stopMotors();
        waitSeconds(0.3);
        double strafeDelta = driver.readPose().getY(DistanceUnit.MM);

        boolean wentRight = askYesNo("PHASE 1: ODOMETRY",
                "The robot just strafed RIGHT at 0.5 power for 0.7 s.",
                "Did it actually strafe to its RIGHT?  A = yes   B = no");

        if (wentRight && strafeDelta > 0) {
            driver.setPodDirections(driver.isXPodForward(), false);
            odometryNote += "; Y pod flipped to REVERSE";
        }

        // Verify the X fix actually took.
        driver.resetPoseToOrigin();
        driver.driveOpenLoop(0.45, 0, 0);
        waitSeconds(0.6);
        driver.stopMotors();
        waitSeconds(0.2);
        if (driver.readPose().getX(DistanceUnit.MM) <= 0) {
            odometryNote += "; X still not increasing - check the pod cable and offset";
            return false;
        }

        driver.resetPoseToOrigin();
        driver.resetHeading();
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
        while (active() && timer.seconds() < 2.0) {
            noise = Math.max(noise, Math.abs(gamepad.left_stick_y));
            noise = Math.max(noise, Math.abs(gamepad.left_stick_x));
            noise = Math.max(noise, Math.abs(gamepad.right_stick_x));
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

        driver.setHoldBrake(false);
        driver.resetPoseToOrigin();

        double found = 0;
        for (double power = 0.02; power <= 0.201; power += 0.02) {
            if (!active()) return;

            driver.driveOpenLoop(power, 0, 0);
            waitSeconds(0.45);
            driver.stopMotors();
            waitSeconds(0.25);

            double moved = Math.abs(driver.readPose().getX(DistanceUnit.MM));
            if (moved >= MOTION_THRESHOLD_MM) {
                found = power;
                break;
            }
        }

        driver.resetPoseToOrigin();
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
            if (!active()) return;

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
        driver.setHoldBrake(false);
        driver.resetHeading();
        driver.resetPoseToOrigin();

        ElapsedTime pushTimer = new ElapsedTime();
        while (active() && pushTimer.seconds() < PUSH_TIMEOUT_SEC) {
            driver.driveOpenLoop(PUSH_POWER, 0, 0);
            if (driver.readPose().getX(DistanceUnit.MM) >= PUSH_TARGET_MM) break;
            idle();
        }
        driver.stopMotors();
        waitSeconds(0.4);

        // Target is where the trial started; the hold has to bring it back from +150 mm.
        driver.setHoldBrake(true);
        driver.setHoldTarget(0, 0, 0);

        double overshoot = 0;
        double finalError = Double.MAX_VALUE;
        ElapsedTime settleTimer = new ElapsedTime();
        while (active() && settleTimer.seconds() < HOLD_SETTLE_SECONDS) {
            driver.drive(1.0);
            double x = driver.getPose().getX(DistanceUnit.MM);
            if (-x > overshoot) overshoot = -x;
            finalError = Math.abs(x);
            idle();
        }

        driver.setHoldBrake(false);
        driver.stopMotors();
        return new TrialResult(finalError, overshoot, finalError <= SETTLED_ERROR_MM);
    }

    // =================================================================
    // PHASE 5 - deadbands from the noise while holding still
    // =================================================================
    private void measureHoldDeadbands() {
        driver.setHoldBrake(false);
        driver.resetPoseToOrigin();
        driver.resetHeading();
        driver.setHoldBrake(true);
        driver.setHoldTarget(0, 0, 0);
        waitSeconds(0.8);

        double maxX = 0;
        double maxY = 0;
        double maxHeading = 0;
        ElapsedTime timer = new ElapsedTime();
        while (active() && timer.seconds() < 2.0) {
            driver.drive(1.0);
            Pose2D pos = driver.getPose();
            maxX = Math.max(maxX, Math.abs(pos.getX(DistanceUnit.MM)));
            maxY = Math.max(maxY, Math.abs(pos.getY(DistanceUnit.MM)));
            maxHeading = Math.max(maxHeading, Math.abs(pos.getHeading(AngleUnit.RADIANS)));
            idle();
        }

        driver.setHoldBrake(false);
        driver.stopMotors();

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
            if (!active()) return;

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
        driver.setHoldBrake(false);
        driver.resetHeading();

        ElapsedTime pushTimer = new ElapsedTime();
        while (active() && pushTimer.seconds() < PUSH_TIMEOUT_SEC) {
            driver.driveOpenLoop(0, 0, HEADING_PUSH_POWER);
            if (Math.abs(driver.readPose().getHeading(AngleUnit.RADIANS)) >= HEADING_PUSH_RAD) break;
            idle();
        }
        driver.stopMotors();
        waitSeconds(0.4);

        // Hold the current position, but the original heading - so only rotation is corrected.
        Pose2D start = driver.readPose();
        driver.setHoldBrake(true);
        driver.setHoldTarget(start.getX(DistanceUnit.MM), start.getY(DistanceUnit.MM), 0);

        double overshoot = 0;
        double finalError = Double.MAX_VALUE;
        ElapsedTime settleTimer = new ElapsedTime();
        while (active() && settleTimer.seconds() < HOLD_SETTLE_SECONDS) {
            driver.drive(1.0);
            double heading = driver.getPose().getHeading(AngleUnit.RADIANS);
            if (-heading > overshoot) overshoot = -heading;
            finalError = Math.abs(heading);
            idle();
        }

        driver.setHoldBrake(false);
        driver.stopMotors();

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

        while (active()) {
            render("PHASE 7: INTAKE",
                    "Now BLOCK the beam with a ball and hold it there.",
                    "Press B when it is blocked.");
            if (gamepad.b) {
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
        while (active() && timer.seconds() < seconds) {
            intake.periodic();
            idle();
        }
        intake.stopMotors();
    }

    /** Ramps one roller until its encoder reports motion; -1 if no encoder is readable. */
    private double findMinSpinPower(boolean lower) {
        for (double power = 0.05; power <= 0.601; power += 0.05) {
            if (!active()) return -1;

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
    /** Draws the values that are now live in the driver and intake. Safe to call repeatedly. */
    public void renderSummary() {
        telemetry.addLine("=== AUTO TUNER: values applied to the live objects ===");
        telemetry.addLine("");

        if (driveTuned) {
            telemetry.addLine("TeleOpDriver.Tuning (in use now):");
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
            telemetry.addLine("Pods: X " + (driver.isXPodForward() ? "FORWARD" : "REVERSE")
                    + ", Y " + (driver.isYPodForward() ? "FORWARD" : "REVERSE"));
            telemetry.addData("Odometry", driver.getOdometryStatus());
            telemetry.addLine("odometry note: " + odometryNote);
            telemetry.addLine("");
        } else {
            telemetry.addLine("Drive not tuned: " + odometryNote);
            telemetry.addLine("");
        }

        if (intake == null) {
            telemetry.addLine("Intake not present - intake phases skipped.");
        } else if (intakeTuned) {
            telemetry.addLine("Intake.Settings (in use now):");
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
            telemetry.addLine("Intake not tuned.");
        }

        telemetry.addLine("");
        telemetry.addLine("Saved to the Control Hub: drive "
                + (driveTuned ? (driveSaved ? "yes" : "FAILED") : "not tuned")
                + ", intake " + (intake == null ? "not present"
                        : intakeTuned ? (intakeSaved ? "yes" : "FAILED") : "not tuned"));
        telemetry.addLine("The next run loads these instead of tuning again.");
        telemetry.update();
    }

    // =================================================================
    // Small helpers
    // =================================================================
    /** One hold trial's score: resting error, overshoot, and whether it settled at all. */
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

    /** The caller decides what "still running" means: after START, or merely not stopped in INIT. */
    private boolean active() {
        return keepRunning.getAsBoolean();
    }

    private void idle() {
        yieldToSdk.run();
    }

    private void render(String... lines) {
        for (String line : lines) {
            telemetry.addLine(line);
        }
        telemetry.update();
    }

    private boolean askYesNo(String title, String line1, String line2) {
        while (active()) {
            render(title, "", line1, "", line2);
            if (gamepad.a) {
                waitButtonRelease();
                return true;
            }
            if (gamepad.b) {
                waitButtonRelease();
                return false;
            }
            idle();
        }
        return false;
    }

    private void waitForA(String title, String line1, String line2) {
        while (active()) {
            render(title, "", line1, line2);
            if (gamepad.a) {
                waitButtonRelease();
                return;
            }
            idle();
        }
    }

    private void waitButtonRelease() {
        ElapsedTime timer = new ElapsedTime();
        while (active() && (gamepad.a || gamepad.b) && timer.seconds() < 2.0) {
            idle();
        }
    }

    private void waitSeconds(double seconds) {
        ElapsedTime timer = new ElapsedTime();
        while (active() && timer.seconds() < seconds) {
            idle();
        }
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
