package org.firstinspires.ftc.teamcode;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter {
    public enum State {
        DEFAULT,
        FLOWER,
        FAILSAFE
    }
    State state = State.DEFAULT;
    public DcMotorEx flywheelMotor, flywheelMotor2;
    public CRServo turretServo, turretServo2, turretServo3;
    public DcMotorEx turretEncoder;
    public Servo hoodServo;
    private PIDf turretPIDf, flywheelPIDf;

    public static final double TURRET_WRAPAROUND = 5000;
    public static final double TURRET_RADIANS_TO_TICKS = 1.0;


    public static final double FLOWER_TURRET_POS = 0.0;
    public static final double FLOWER_FLYWHEEL_VELOCITY = 700;
    public static final double FLOWER_HOOD_ANGLE = 20;


    public static final double FAILSAFE_TURRET_POS = 0.0;
    public static final double FAILSAFE_FLYWHEEL_VELOCITY = 1000;
    public static final double FAILSAFE_HOOD_ANGLE = 20;

    double UPPER_ANGLE = 25;
    double UPPER_POS = 1.0;
    double LOWER_ANGLE = 10;
    double LOWER_POS = 0.0;

    public double hood_angle_to_pos(double angle) {
        return LOWER_POS + (angle - LOWER_ANGLE) * ((UPPER_POS - LOWER_POS) / (UPPER_ANGLE - LOWER_ANGLE);
    }

    Sotm sotm = new Sotm();

    public void setTurretPower(Double power) {
        turretServo.setPower(power);
        turretServo2.setPower(power);
        turretServo3.setPower(power);
    }

    public void setFlywheelPower(Double power) {
        flywheelMotor.setPower(power);
        flywheelMotor2.setPower(power);
    }


    public Shooter(HardwareMap hardwareMap, Boolean turret_reset) {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        turretServo = hardwareMap.get(CRServo.class, "turret1");
        turretServo2 = hardwareMap.get(CRServo.class, "turret2");
        turretServo3 = hardwareMap.get(CRServo.class, "turret3");
        turretEncoder = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        hoodServo = hardwareMap.get(Servo.class, "hood");
        if (turret_reset) {turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);}


        flywheelMotor2.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void initLoopShooter() {
        turretPIDf = new PIDf(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        flywheelPIDf = new PIDf(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    public void loopShooter(Follower follower) {
        double globalTurretPos = turretEncoder.getCurrentPosition() + follower.pose().heading() * TURRET_RADIANS_TO_TICKS;
        double targetTurretPos;
        double targetFlywheelVelocity;
        double hoodAngle;
        switch (state) {
            case FLOWER:
                targetTurretPos = FLOWER_TURRET_POS;
                targetFlywheelVelocity = FLOWER_FLYWHEEL_VELOCITY;
                hoodAngle = FLOWER_HOOD_ANGLE;
                break;
            case FAILSAFE:
                targetTurretPos =   FAILSAFE_TURRET_POS;
                targetFlywheelVelocity = FAILSAFE_FLYWHEEL_VELOCITY;
                hoodAngle = FAILSAFE_HOOD_ANGLE;
                break;
            default:
                targetTurretPos = Math.min(Math.max(sotm.yaw() * TURRET_RADIANS_TO_TICKS, -TURRET_WRAPAROUND), TURRET_WRAPAROUND);
                targetFlywheelVelocity = sotm.velocity();
                hoodAngle = sotm.hood_angle();
                break;
        }
        hoodServo.setPosition(hood_angle_to_pos(hoodAngle));
        setFlywheelPower(flywheelPIDf.output(flywheelMotor.getVelocity(), targetFlywheelVelocity));
        setTurretPower(turretPIDf.output(globalTurretPos, targetTurretPos));
    }

}
