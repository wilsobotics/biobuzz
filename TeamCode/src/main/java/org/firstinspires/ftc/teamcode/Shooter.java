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
    private PIDf turretPIDf, flywheelPIDf;

    public void setTurretPower(Double power) {
        turretServo.setPower(power);
        turretServo2.setPower(power);
        turretServo3.setPower(power);
    }

    public void setFlywheelPower(Double power) {
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
        turretPIDf = new PIDf(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        flywheelPIDf = new PIDf(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    public void loopShooter() {
        setFlywheelPower(flywheelPIDf.output(flywheelMotor.getVelocity(), 1000.0));
        setTurretPower(turretPIDf.output((double) turretEncoder.getCurrentPosition(), 0.0));
    }

}
