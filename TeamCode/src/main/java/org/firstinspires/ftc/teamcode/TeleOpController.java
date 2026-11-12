package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Kindly rotate your wheels in accordance to my controller actions")
public class TeleOpController extends OpMode {
    TeleOpDriver driveTrain = new TeleOpDriver();

    Boolean LastCheck = Boolean.FALSE;
    Boolean Current = Boolean.FALSE;

    Boolean L1LastCheck = Boolean.FALSE;
    Boolean L1Current = Boolean.FALSE;

    @Override
    public void init() {
        driveTrain.gamepad1 = gamepad1;
        driveTrain.intialiseDriver();
    }

    @Override
    public void loop() {
        // Default cap of 0.75 power normally, removed (1.0) while R2 (right_trigger) is held down
        double speedCap = 0.75;
        if (gamepad1.left_trigger > 0.5) {
            speedCap = 1.0;
        }

        // Reset heading check
        Current = gamepad1.y;
        if (Current && !LastCheck) {
            driveTrain.resetHeading();
        }
        LastCheck = Current;

        // Hold Break toggle check (L1 / left_bumper)
        L1Current = gamepad1.left_bumper;
        if (L1Current && !L1LastCheck) {
            driveTrain.toggleHoldBreak();
        }
        L1LastCheck = L1Current;

        // Run drive loop with the dynamic speed multiplier
        driveTrain.DriveLoop(speedCap);
    }
}

/*
* Controls:
* Y button - zero Field-centric drive direction and coordinates
* L2 Held more than half-way - removes 0.75 "precision" speed cap and allows you to reach full speed
* R2 (Reserved not implemented yet) - Shoot Balls
* R1 (Reserved not implemented yet) - Toggle on Intake
* Left Joystick Y - Forwards and backwards movement
* Left Joystick X - Left and Right Strafing
* Right Joystick X - Rotation Clockwise and Counter-Clockwise
* L1 - Hold Break Toggle (Active Position Hold via Odometry)
* 
* */
