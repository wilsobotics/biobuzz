package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.ArrayList;
import java.util.List;

/**
 * Measures SOME of the tuning values used by TeleOpDriver, TeleOpController and Intake, then
 * writes them into DriveConfig and IntakeConfig - the two @Config classes the Slothboard edits
 * live - and prints them so you can paste them into the defaults. Nothing is written to flash:
 * the values hold for the rest of the app session, so restart the app if you want the code
 * defaults back.
 *
 * This OpMode is named "partial" on purpose: it measures about ten numbers, not all of them.
 *
 * WHAT IT ACTUALLY MEASURES (no human judgement needed):
 *   - joystick deadbands      from the sticks' own noise floor
 *   - MIN_HOLD_POWER          the power at which the robot first breaks static friction
 *   - KP_TRANSLATIONAL        sweep of candidate gains, scored on overshoot + settling error
 *   - KP_HEADING              same, for rotation
 *   - hold deadbands          from the position/heading noise while holding still
 *   - beam-break polarity     from the sensor's clear vs blocked levels
 *   - minimum roller power    the power at which each roller encoder first reads motion
 *   - roller direction        confirmed by you watching the balls, then applied
 *   - pod directions          from which way the odometry counts while the robot drives
 *
 * WHAT IT CANNOT MEASURE (a human decides):
 *   - pod offsets             a tape measure, not a sensor (verifies the result only)
 *   - starting pose           depends on where you place the robot
 *   - MAX_HOLD_POWER          a safety ceiling, not a physical property
 *   - intake / eject / shoot powers, door positions, MAX_BALLS, cooldown, eject time
 *                             depend on how your robot feeds and on game strategy
 *   - the controller's speed constants
 *
 * The robot drives itself during phases 1-6, so give it about 1 m of clear floor in front and
 * keep hands off the controller until it asks. There is a "press A to continue" gate before every
 * phase, and STOP aborts at any point.
 */
@TeleOp(name = "Auto Tuner (partial)", group = "Tuning")
public class AutoTuner extends LinearOpMode {

    // ---- how hard the tests push ----
    private static final double PUSH_POWER = 0.45;
    private static final double PUSH_TARGET_MM = 150.0;
    private static final double PUSH_TIMEOUT_SEC = 2.0;
    public static final double INTAKE_SECONDS = 2.5;
    private static final double HOLD_SETTLE_SECONDS = 2.5;
    public static final double ODO_TUNING_FORWARD = 0.5;
    private static final double HEADING_PUSH_POWER = 0.5;
    private static final double HEADING_PUSH_RAD = Math.toRadians(25);
    private static final double MOTION_THRESHOLD_MM = 3.0;
    private static final double SETTLED_ERROR_MM = 30.0;
    private static final double SETTLED_ERROR_DEG = 3.0;
    private static final double ROLLER_MOTION_TICKS_PER_SEC = 20.0;

    /** Score is the mean error over this trailing window, not one instantaneous sample. */
    private static final long SCORE_WINDOW_NS = 500_000_000L;
    /** A candidate must beat the incumbent by this much, or the two are treated as a tie. */
    private static final double SCORE_TIE_EPSILON = 1.0;
    /** Below this rotation a heading trial has no error to score, so it is not a measurement. */
    private static final double HEADING_MIN_ACHIEVED_RAD = Math.toRadians(15.0);
    private static final int HEADING_ATTEMPTS_MAX = 2;
    private static final long BUTTON_DEBOUNCE_MS = 150;

    private static final double[] TRANSLATIONAL_CANDIDATES =
            {0.0015, 0.0025, 0.0035, 0.0050, 0.0070, 0.0100};
    private static final double[] HEADING_CANDIDATES =
            {0.5, 0.8, 1.25, 1.8, 2.5};
    public static final double ODO_TUNING_SECONDS = 0.7;

    private TeleOpDriver driveTrain;
    private Intake intake;

    private boolean driveReady = false;
    private boolean intakeReady = false;
    private String driveError = "";
    private String intakeError = "";

    private boolean odometryOk = false;
    private String odometryNote = "not checked";
    private String beamNote = "not checked";
    private String directionNote = "not checked";
    private String translationNote = "not run";
    private String headingNote = "not run";
    private double lowerMinSpin = -1;
    private double upperMinSpin = -1;

    /**
     * Overshoot, mean error and settling verdict from one hold trial. An invalid trial carries a
     * reason instead of numbers and must never be scored.
     */
    private static class TrialResult {
        final boolean valid;
        final String invalidReason;
        final double meanError;
        final double overshoot;
        final boolean settled;

        TrialResult(double meanError, double overshoot, boolean settled) {
            this.valid = true;
            this.invalidReason = "";
            this.meanError = meanError;
            this.overshoot = overshoot;
            this.settled = settled;
        }

        private TrialResult(String reason) {
            this.valid = false;
            this.invalidReason = reason;
            this.meanError = Double.MAX_VALUE;
            this.overshoot = Double.MAX_VALUE;
            this.settled = false;
        }

        static TrialResult invalid(String reason) {
            return new TrialResult(reason);
        }

        double score() {
            if (!valid) return Double.MAX_VALUE;
            return meanError + 2.0 * overshoot + (settled ? 0 : 1000.0);
        }
    }

    /**
     * Trailing 0.5 s of error samples. A single sample at the deadline rewards whichever gain
     * happened to be mid-oscillation, so the score is averaged over a window and "settled" means
     * every sample in that window stayed inside the deadband.
     */
    private static class ErrorWindow {
        private static class Sample {
            final long tNs;
            final double err;

            Sample(long tNs, double err) {
                this.tNs = tNs;
                this.err = err;
            }
        }

        private final List<Sample> samples = new ArrayList<>();

        void add(double error) {
            long now = System.nanoTime();
            samples.add(new Sample(now, Math.abs(error)));
            while (!samples.isEmpty() && now - samples.get(0).tNs > SCORE_WINDOW_NS) {
                samples.remove(0);
            }
        }

        boolean hasData() {
            return !samples.isEmpty();
        }

        double mean() {
            if (samples.isEmpty()) return Double.MAX_VALUE;
            double sum = 0;
            for (Sample sample : samples) sum += sample.err;
            return sum / samples.size();
        }

        double max() {
            double worst = 0;
            for (Sample sample : samples) worst = Math.max(worst, sample.err);
            return worst;
        }

        /** True only if the error stayed inside the deadband for the whole window. */
        boolean stayedInside(double deadband) {
            return hasData() && max() <= deadband;
        }
    }

    /** YES / NO, or ABORT when STOP is pressed mid-question. */
    private enum Answer { YES, NO, ABORT }

    @Override
    public void runOpMode() {
        // Everything this OpMode writes goes to the Driver Station AND the Slothboard, so the
        // dashboard shows the same phase gates, live trials and results the driver sees.
        telemetry = new MultipleTelemetry(telemetry,
                FtcDashboard.getInstance().getTelemetry());

        renderIntro();

        driveTrain = new TeleOpDriver(hardwareMap, telemetry, gamepad1);
        driveTrain.setTelemetryEnabled(false);
        try {
            driveTrain.initialiseDriver();
            driveReady = true;
        } catch (RuntimeException e) {
            driveError = e.getClass().getSimpleName() + ": " + e.getMessage();
        }

        try {
            intake = new Intake(hardwareMap);
            if (intake.isAvailable()) {
                intake.stopMotors();
                intakeReady = true;
            } else {
                intakeError = intake.getFault();
            }
        } catch (RuntimeException e) {
            intakeError = e.getClass().getSimpleName() + ": " + e.getMessage();
        }

        waitForStart();
        if (!opModeIsActive()) {
            return;
        }

        // Nothing moves until the operator has confirmed the floor is clear.
        if (pressAToContinue("READY?",
                "The robot will drive itself for ~90-120 s,",
                "~150 mm per trial, with no wall or distance sensing.",
                "",
                "Clear 1 m of floor ahead. Hands off the sticks.",
                "STOP aborts at any point.")) {
            runTuning();
        }

        parkRobot();
        showSummaryUntilStopped();
    }

    private void renderIntro() {
        render("AUTO TUNER (PARTIAL)",
                "",
                "TUNED HERE: joystick deadbands, minHoldPower,",
                "kpTranslational, kpHeading, hold deadbands,",
                "beam polarity, roller directions, roller floor,",
                "pod directions.",
                "",
                "NOT TUNED (set these by hand): maxHoldPower,",
                "intakePower, fullPower, reversePower, ejectPower,",
                "shootPower, doorOpen, doorClosed, maxBalls,",
                "countCooldownSeconds, ejectTimeSeconds, and the",
                "controller's speed constants.",
                "",
                "MOTION: ~90-120 s of driving, no wall sensing.",
                "",
                "Press START, then A at each phase gate.");
    }

    // =================================================================
    // Phase sequencing - one gate before every phase
    // =================================================================
    private void runTuning() {
        if (driveReady) {
            if (!pressAToContinue("PHASE 1: ODOMETRY",
                    "Drives forward 0.7 s, then asks whether it really",
                    "moved forward. Your answers can flip a pod direction.")) {
                return;
            }
            odometryOk = checkOdometry();

            if (odometryOk) {
                if (!pressAToContinue("PHASE 2: JOYSTICK DEADBAND",
                        "Measuring the sticks' own noise floor.",
                        "Hands off BOTH sticks for about 3 s.")) {
                    return;
                }
                measureStickDeadband();

                if (!pressAToContinue("PHASE 3: FRICTION FLOOR",
                        "Ramps wheel power from 0.02 to 0.20.",
                        "The robot starts moving partway through.")) {
                    return;
                }
                measureFrictionFloor();

                if (!pressAToContinue("PHASE 4: KP_TRANSLATIONAL",
                        "6 candidates, each pushed ~150 mm forward",
                        "and left 2.5 s to settle.")) {
                    return;
                }
                tuneTranslationalGain();

                if (!pressAToContinue("PHASE 5: HOLD DEADBANDS",
                        "Holds one pose for 2 s. Do not touch the robot:",
                        "its own noise while holding is the measurement.")) {
                    return;
                }
                measureHoldDeadbands();

                if (!pressAToContinue("PHASE 6: KP_HEADING",
                        "5 candidates, each spun ~25 deg",
                        "and left 2.5 s to settle.")) {
                    return;
                }
                tuneHeadingGain();
            }

            driveTrain.setHoldBrake(false);
            driveTrain.stopMotors();
        }

        if (intakeReady) {
            if (!pressAToContinue("PHASE 7: INTAKE SENSORS",
                    "Beam-break must be CLEAR (no ball in it).",
                    "The rollers will spin - watch which way the balls go.")) {
                return;
            }
            tuneIntake();
        }
    }

    private void parkRobot() {
        if (driveReady) {
            driveTrain.setHoldBrake(false);
            driveTrain.stopMotors();
        }
        if (intakeReady) {
            intake.stopMotors();
            intake.stop();
        }
    }

    // =================================================================
    // PHASE 1 - odometry sanity, and pod direction discovery
    // =================================================================
    private boolean checkOdometry() {
        if (!driveTrain.isOdometryReady()) {
            odometryNote = "Pinpoint not READY (" + driveTrain.getOdometryStatus() + ")";
            return false;
        }

        driveTrain.setPodDirections(true, true);
        driveTrain.resetPoseToOrigin();

        // Drive forward and ask the operator whether the robot really went forward. Without that
        // answer a negative X reading could mean "pod backwards" or "motors backwards", and
        // flipping the wrong one hides a wiring fault.
        driveTrain.driveOpenLoop(ODO_TUNING_FORWARD, 0, 0);
        waitSeconds(ODO_TUNING_SECONDS);
        driveTrain.stopMotors();
        waitSeconds(0.3);
        double forwardDelta = driveTrain.readPose().getX(DistanceUnit.MM);

        Answer forwardAnswer = askAnswer("PHASE 1: ODOMETRY",
                "The robot just drove forward at 0.5 power for 0.7 s.",
                "Did it move FORWARD?   A = yes    B = no");

        if (forwardAnswer == Answer.ABORT) {
            odometryNote = "aborted during phase 1 - pod directions left as they were";
            return false;
        }
        if (forwardAnswer == Answer.NO) {
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

        Answer rightAnswer = askAnswer("PHASE 1: ODOMETRY",
                "The robot just strafed RIGHT at 0.5 power for 0.7 s.",
                "Did it strafe to its RIGHT?   A = yes    B = no");

        if (rightAnswer == Answer.ABORT) {
            odometryNote += "; aborted before the strafe check - Y pod left as it was";
            return false;
        }
        if (rightAnswer == Answer.YES && strafeDelta > 0) {
            driveTrain.setPodDirections(driveTrain.isXPodForward(), false);
            odometryNote += "; Y pod flipped to REVERSE";
        }

        if (rightAnswer == Answer.NO) {
            odometryNote += "; strafe direction unconfirmed - Y pod left as it was";
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
        DriveConfig.joystickEngageDeadband = engage;
        DriveConfig.joystickReleaseDeadband = release;

        if (noise > 0.15) {
            render("Stop it....",
                    "Just get your fingers off the joystick",
                    "please...");
            render("Waiting 5 seconds for retry");
            waitSeconds(5.0);
            render("Retrying...");
            measureStickDeadband();

        }

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
        DriveConfig.minHoldPower = found > 0 ? Math.min(0.15, found + 0.01) : 0.06;

        render("PHASE 3: FRICTION FLOOR",
                found > 0 ? String.format("first movement at %.2f power", found)
                          : "no movement measured - keeping default",
                String.format("MIN_HOLD_POWER = %.3f", DriveConfig.minHoldPower));
        waitSeconds(1.2);
    }

    // =================================================================
    // PHASE 4 - translational gain sweep
    // =================================================================
    private void tuneTranslationalGain() {
        double bestGain = DriveConfig.kpTranslational;
        double bestScore = Double.MAX_VALUE;
        boolean scoredAnything = false;
        int rejected = 0;

        for (double candidate : TRANSLATIONAL_CANDIDATES) {
            if (!opModeIsActive()) {
                DriveConfig.kpTranslational = bestGain;   // never leave a half-tested candidate behind
                return;
            }

            DriveConfig.kpTranslational = candidate;
            TrialResult result = runTranslationTrial();

            if (!result.valid) {
                rejected++;
                render("PHASE 4: KP_TRANSLATIONAL",
                        String.format("candidate %.4f", candidate),
                        "trial rejected: " + result.invalidReason,
                        "",
                        String.format("rejected so far %d", rejected));
                continue;
            }

            if (result.score() < bestScore - SCORE_TIE_EPSILON) {
                bestScore = result.score();
                bestGain = candidate;
            }
            scoredAnything = true;

            render("PHASE 4: KP_TRANSLATIONAL",
                    String.format("candidate %.4f", candidate),
                    String.format("mean error %.1f mm", result.meanError),
                    String.format("overshoot  %.1f mm", result.overshoot),
                    result.settled ? "settled" : "did not settle",
                    "",
                    String.format("best so far %.4f", bestGain));
        }

        DriveConfig.kpTranslational = bestGain;
        translationNote = scoredAnything
                ? String.format("swept 6, chose %.4f (%d rejected)", bestGain, rejected)
                : "no usable trial - kept the value from the code";

        render("PHASE 4: KP_TRANSLATIONAL",
                String.format("chosen KP_TRANSLATIONAL = %.4f", bestGain),
                translationNote);
        waitSeconds(1.2);
    }

    /** Pushes the robot off target, then lets the hold drive it back and scores the recovery. */
    private TrialResult runTranslationTrial() {
        driveTrain.setHoldBrake(false);
        driveTrain.resetHeading();
        driveTrain.resetPoseToOrigin();
        driveTrain.consumeStickBreach();   // drop anything left over from the previous trial

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

        ErrorWindow window = new ErrorWindow();
        double overshoot = 0;
        ElapsedTime settleTimer = new ElapsedTime();
        while (opModeIsActive() && settleTimer.seconds() < HOLD_SETTLE_SECONDS) {
            driveTrain.drive(1.0);

            if (driveTrain.consumeStickBreach()) {
                driveTrain.setHoldBrake(false);
                driveTrain.stopMotors();
                return TrialResult.invalid("stick moved during the hold");
            }

            double x = driveTrain.getPose().getX(DistanceUnit.MM);
            if (-x > overshoot) overshoot = -x;
            window.add(x);
            idle();
        }

        driveTrain.setHoldBrake(false);
        driveTrain.stopMotors();
        return new TrialResult(window.mean(), overshoot, window.stayedInside(SETTLED_ERROR_MM));
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
        DriveConfig.holdPositionDeadbandMm =
                clamp(translationNoise + 3.0, 5.0, 30.0);
        DriveConfig.holdHeadingDeadbandRad =
                clamp(maxHeading + Math.toRadians(0.5), Math.toRadians(1.0), Math.toRadians(6.0));

        render("PHASE 5: HOLD DEADBANDS",
                String.format("position noise %.1f mm", translationNoise),
                String.format("heading noise  %.2f deg", Math.toDegrees(maxHeading)),
                String.format("position deadband %.1f mm", DriveConfig.holdPositionDeadbandMm),
                String.format("heading deadband  %.2f deg",
                        Math.toDegrees(DriveConfig.holdHeadingDeadbandRad)));
        waitSeconds(1.2);
    }

    // =================================================================
    // PHASE 6 - heading gain sweep
    // =================================================================
    private void tuneHeadingGain() {
        double bestGain = DriveConfig.kpHeading;
        double bestScore = Double.MAX_VALUE;
        boolean scoredAnything = false;

        for (double candidate : HEADING_CANDIDATES) {
            if (!opModeIsActive()) {
                DriveConfig.kpHeading = bestGain;   // never leave a half-tested candidate behind
                return;
            }

            DriveConfig.kpHeading = candidate;

            // A trial that barely rotated cannot separate the candidates, so retry it once and
            // then give up rather than score noise.
            TrialResult result = TrialResult.invalid("not run");
            for (int attempt = 1; attempt <= HEADING_ATTEMPTS_MAX && opModeIsActive(); attempt++) {
                result = runHeadingTrial();
                if (result.valid) break;

                render("PHASE 6: KP_HEADING",
                        String.format("candidate %.2f", candidate),
                        "trial rejected: " + result.invalidReason,
                        String.format("retry %d of %d", attempt, HEADING_ATTEMPTS_MAX));
                waitSeconds(1.0);
            }

            if (!result.valid) {
                headingNote = "NOT tuned - " + result.invalidReason
                        + "; keeping the value from the code";
                break;
            }

            if (result.score() < bestScore - SCORE_TIE_EPSILON) {
                bestScore = result.score();
                bestGain = candidate;
            }
            scoredAnything = true;

            render("PHASE 6: KP_HEADING",
                    String.format("candidate %.2f", candidate),
                    String.format("mean error %.2f deg", result.meanError),
                    String.format("overshoot  %.2f deg", result.overshoot),
                    result.settled ? "settled" : "did not settle",
                    "",
                    String.format("best so far %.2f", bestGain));
        }

        DriveConfig.kpHeading = bestGain;
        if (scoredAnything) {
            headingNote = String.format("swept, chose %.2f", bestGain);
        }

        render("PHASE 6: KP_HEADING",
                String.format("chosen KP_HEADING = %.2f", bestGain),
                headingNote);
        waitSeconds(1.2);
    }

    /** Spins the robot off heading, then holds position while the heading is driven back to 0. */
    private TrialResult runHeadingTrial() {
        driveTrain.setHoldBrake(false);
        driveTrain.resetHeading();
        driveTrain.consumeStickBreach();

        ElapsedTime pushTimer = new ElapsedTime();
        while (opModeIsActive() && pushTimer.seconds() < PUSH_TIMEOUT_SEC) {
            driveTrain.driveOpenLoop(0, 0, HEADING_PUSH_POWER);
            if (Math.abs(driveTrain.readPose().getHeading(AngleUnit.RADIANS)) >= HEADING_PUSH_RAD) break;
            idle();
        }
        driveTrain.stopMotors();
        waitSeconds(0.4);

        Pose2D start = driveTrain.readPose();
        double achieved = Math.abs(start.getHeading(AngleUnit.RADIANS));
        if (achieved < HEADING_MIN_ACHIEVED_RAD) {
            // With a tiny starting error every candidate scores the same, so this is not a trial.
            return TrialResult.invalid(String.format(
                    "robot only reached %.1f deg of the %.0f deg push",
                    Math.toDegrees(achieved), Math.toDegrees(HEADING_PUSH_RAD)));
        }

        // Hold the current position, but the original heading - so only rotation is corrected.
        driveTrain.setHoldBrake(true);
        driveTrain.setHoldTarget(start.getX(DistanceUnit.MM), start.getY(DistanceUnit.MM), 0);

        ErrorWindow window = new ErrorWindow();
        double overshoot = 0;
        ElapsedTime settleTimer = new ElapsedTime();
        while (opModeIsActive() && settleTimer.seconds() < HOLD_SETTLE_SECONDS) {
            driveTrain.drive(1.0);

            if (driveTrain.consumeStickBreach()) {
                driveTrain.setHoldBrake(false);
                driveTrain.stopMotors();
                return TrialResult.invalid("stick moved during the hold");
            }

            double heading = driveTrain.getPose().getHeading(AngleUnit.RADIANS);
            if (-heading > overshoot) overshoot = -heading;
            window.add(Math.toDegrees(heading));
            idle();
        }

        driveTrain.setHoldBrake(false);
        driveTrain.stopMotors();

        return new TrialResult(window.mean(), Math.toDegrees(overshoot),
                window.stayedInside(SETTLED_ERROR_DEG));
    }

    // =================================================================
    // PHASE 7 - intake sensors, roller direction and roller floor
    // =================================================================
    private void tuneIntake() {
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
            IntakeConfig.beamActiveLow = !blockedRaw;
            beamNote = "blocked reads " + (blockedRaw ? "HIGH" : "LOW")
                    + " -> beamActiveLow = " + IntakeConfig.beamActiveLow;
        }

        lowerMinSpin = findMinSpinPower(true);
        upperMinSpin = findMinSpinPower(false);

        // Direction confirmation: run the intake and watch which way the balls go.
        runIntakeFor(INTAKE_SECONDS);
        Answer inward = askAnswer("PHASE 7: INTAKE",
                "The rollers just ran INTAKE for 2.5 s.",
                "Did they pull the balls INWARD?   A = yes    B = no");

        if (inward == Answer.YES) {
            directionNote = "roller directions left as they were";
        } else if (inward == Answer.NO) {
            IntakeConfig.lowerReversed = !IntakeConfig.lowerReversed;
            IntakeConfig.upperReversed = !IntakeConfig.upperReversed;
            intake.applyDirections();
            runIntakeFor(INTAKE_SECONDS);
            directionNote = "both roller directions flipped in IntakeConfig";
        } else {
            // Aborting here must not be read as "no": that would flip both rollers and apply them.
            intake.stopMotors();
            intake.stop();
            directionNote = "aborted - roller directions left UNCHANGED";
            render("PHASE 7: INTAKE", directionNote);
            waitSeconds(1.0);
            return;
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
        intake.setTestPower(IntakeConfig.intakePower, IntakeConfig.intakePower);
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < seconds) idle();
        intake.stopMotors();
    }

    /**
     * Ramps one roller until its encoder reports motion; -1 if no encoder is readable. The roller
     * motors run RUN_WITHOUT_ENCODER, so this is reading the encoder's measured velocity, which is
     * only real if a roller encoder is actually wired up. A -1 means "set this one by eye".
     */
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
                telemetry.addLine("DriveConfig (the dashboard shows these live):");
                telemetry.addData("kpTranslational", "%.4f", DriveConfig.kpTranslational);
                telemetry.addData("kpHeading", "%.2f", DriveConfig.kpHeading);
                telemetry.addData("minHoldPower", "%.3f", DriveConfig.minHoldPower);
                telemetry.addData("holdPositionDeadbandMm", "%.1f", DriveConfig.holdPositionDeadbandMm);
                telemetry.addData("holdHeadingDeadbandRad", "%.4f (%.2f deg)",
                        DriveConfig.holdHeadingDeadbandRad,
                        Math.toDegrees(DriveConfig.holdHeadingDeadbandRad));
                telemetry.addData("joystickEngageDeadband", "%.3f", DriveConfig.joystickEngageDeadband);
                telemetry.addData("joystickReleaseDeadband", "%.3f", DriveConfig.joystickReleaseDeadband);
                telemetry.addData("maxHoldPower", "%.2f (unchanged, safety cap)", DriveConfig.maxHoldPower);
                telemetry.addLine("Pods: X " + (driveTrain.isXPodForward() ? "FORWARD" : "REVERSE")
                        + ", Y " + (driveTrain.isYPodForward() ? "FORWARD" : "REVERSE"));
                telemetry.addData("Odometry", driveTrain.getOdometryStatus());
                telemetry.addLine("odometry note: " + odometryNote);
                telemetry.addLine("kpTranslational: " + translationNote);
                telemetry.addLine("kpHeading: " + headingNote);
                telemetry.addLine("");
            } else {
                telemetry.addLine("Driver unavailable: " + driveError);
                telemetry.addLine("");
            }

            if (intakeReady) {
                telemetry.addLine("IntakeConfig (measured entries only):");
                telemetry.addData("beamActiveLow", "%b", IntakeConfig.beamActiveLow);
                telemetry.addData("lowerReversed", "%b", IntakeConfig.lowerReversed);
                telemetry.addData("upperReversed", "%b", IntakeConfig.upperReversed);
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

    /**
     * Blocks until A (continue) or B (abort) is pressed, and returns false on abort or STOP.
     * Used before every phase so nothing moves until the operator says the field is clear.
     *
     * The first loop deliberately waits for a held button to be released: without it, one long
     * press of A would run straight through every gate in the tune.
     */
    private boolean pressAToContinue(String title, String... body) {
        ElapsedTime release = new ElapsedTime();
        while (opModeIsActive() && (gamepad1.a || gamepad1.b) && release.seconds() < 2.0) {
            render(title, "", "Release the button, then choose.");
            idle();
        }

        while (opModeIsActive()) {
            telemetry.addLine("== " + title + " ==");
            for (String line : body) {
                telemetry.addLine(line);
            }
            telemetry.addLine("");
            telemetry.addLine("A = continue    B = abort");
            telemetry.update();

            if (gamepad1.a) {
                waitButtonRelease();
                return true;
            }
            if (gamepad1.b) {
                telemetry.addLine("Aborted by driver.");
                telemetry.update();
                waitButtonRelease();
                return false;
            }
            idle();
        }
        return false;
    }

    /**
     * A real question with three outcomes. ABORT is not the same as NO: the caller must leave
     * hardware alone rather than treating a stopped OpMode as an answer.
     */
    private Answer askAnswer(String title, String line1, String line2) {
        ElapsedTime release = new ElapsedTime();
        while (opModeIsActive() && (gamepad1.a || gamepad1.b) && release.seconds() < 2.0) {
            render(title, "", "Release the button, then answer.");
            idle();
        }

        while (opModeIsActive()) {
            render(title, "", line1, "", line2);
            if (gamepad1.a) {
                waitButtonRelease();
                return Answer.YES;
            }
            if (gamepad1.b) {
                waitButtonRelease();
                return Answer.NO;
            }
            idle();
        }
        return Answer.ABORT;
    }

    private void waitButtonRelease() {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && (gamepad1.a || gamepad1.b) && timer.seconds() < 2.0) {
            idle();
        }
        // Give the Driver Station a moment to see the edge, so the next gate starts un-pressed.
        sleep(BUTTON_DEBOUNCE_MS);
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
