package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class TestIntake extends OpMode {

    DcMotor motor;

    @Override
    public void init() {

        motor = hardwareMap.get(DcMotor.class, "intake");
    }

    @Override
    public void loop() {
        motor.setPower(-gamepad1.right_trigger);
    }
}
