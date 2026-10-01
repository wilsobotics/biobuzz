package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Tip Detects", group = "Vision")
public class TipDetectsTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Instantiate the updated class with the active alliance
        TipDetects vision = new TipDetects(hardwareMap, "limelight", TipDetects.Alliance.RED);

        telemetry.addLine("Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            vision.update();

            telemetry.addData("Hive State", vision.getState().toString());
            telemetry.addData("Seen Tags", vision.getDetectedClusterTags());
            telemetry.addData("Watchdog Time (s)", "%.2f", vision.getWatchdogTime());

            telemetry.update();
        }

        vision.stop();
    }
}