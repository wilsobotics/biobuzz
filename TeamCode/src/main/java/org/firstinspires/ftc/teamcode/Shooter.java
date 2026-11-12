package org.firstinspires.ftc.teamcode;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    public enum State {
        DEFAULT,
        FLOWER,
        FAILSAFE
    }
    public DcMotorEx flywheelMotor, flywheelMotor2;
    public CRServo turretServo, turretServo2, turretServo3;
    public DcMotorEx turretEncoder;

    public void setTurretpower(Double power) {
        turretServo.setPower(power);
        turretServo2.setPower(power);
        turretServo3.setPower(power);
    }

    public void setFlywheelpower(Double power) {
        flywheelMotor.setPower(power);
        flywheelMotor2.setPower(power);
    }


    public Shooter(HardwareMap hardwareMap) {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        turretServo = hardwareMap.get(CRServo.class, "turret1");
        turretServo2 = hardwareMap.get(CRServo.class, "turret2");
        turretServo3 = hardwareMap.get(CRServo.class, "turret3");
        turretEncoder = hardwareMap.get(DcMotorEx.class, "FrontLeft");

        flywheelMotor2.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void initLoopShooter() {

    }

    public void loopShooter() {

    }

}
