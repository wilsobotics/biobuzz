package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Runs the AutoTuner on its own, for bench sessions where you do not want to enter a match.
 *
 * This is a thin wrapper: the measurements, the prompts and the values all belong to AutoTuner,
 * and the TeleOpDriver and Intake it tunes here are the same classes (and the same Tuning /
 * Settings fields) TeleOpController uses. Nothing is printed for you to retype.
 */
@TeleOp(name = "Auto Tuner", group = "Tuning")
public class AutoTunerOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry.addLine("AUTO TUNER");
        telemetry.addLine("");
        telemetry.addLine("The robot will drive itself. Put it on the field with");
        telemetry.addLine("about 1 m of clear floor ahead, hands off the sticks.");
        telemetry.addLine("");
        telemetry.addLine("It measures the hold-brake gains, deadbands, the");
        telemetry.addLine("friction floor and the intake sensors, and writes the");
        telemetry.addLine("results into the driver and intake it hands them to.");
        telemetry.addLine("");
        telemetry.addLine("Press START to begin. STOP aborts at any point.");
        telemetry.update();

        TeleOpDriver driveTrain = null;
        Intake intake = null;
        String problem = "";

        try {
            driveTrain = new TeleOpDriver(hardwareMap, telemetry, gamepad1);
            driveTrain.setTelemetryEnabled(false);
            driveTrain.initialiseDriver();
        } catch (RuntimeException e) {
            problem = "Drive unavailable: " + e.getClass().getSimpleName() + ": " + e.getMessage();
        }

        try {
            intake = new Intake(hardwareMap);
            intake.stopMotors();
            intake.stop();
        } catch (RuntimeException e) {
            String intakeProblem = "Intake unavailable: "
                    + e.getClass().getSimpleName() + ": " + e.getMessage();
            problem = problem.isEmpty() ? intakeProblem : problem + " | " + intakeProblem;
            intake = null;
        }

        if (driveTrain == null) {
            // The drive train owns the odometry and the Tuning object, so there is no point
            // running the intake-only half of the tune without it.
            telemetry.addLine(problem);
            telemetry.addLine("");
            telemetry.addLine("The tuner needs the drive train. Check the configuration.");
            telemetry.update();

            waitForStart();
            while (opModeIsActive()) {
                idle();
            }
            return;
        }

        AutoTuner tuner = new AutoTuner(driveTrain, intake, gamepad1, telemetry,
                this::opModeIsActive, this::idle);

        if (!problem.isEmpty()) {
            telemetry.addLine(problem);
            telemetry.update();
        }

        waitForStart();
        if (!opModeIsActive()) {
            return;
        }

        tuner.run();

        // Leave the robot safe, then hold the results on screen until the operator stops.
        driveTrain.setHoldBrake(false);
        driveTrain.stopMotors();
        if (intake != null) {
            intake.stopMotors();
            intake.stop();
        }

        while (opModeIsActive()) {
            tuner.renderSummary();
            idle();
        }
    }
}
