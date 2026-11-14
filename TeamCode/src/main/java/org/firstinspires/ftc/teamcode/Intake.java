package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// =====================================================================
// HOW TO USE (from your OpMode):
//   Intake intake = new Intake(hardwareMap);   // once, in init
//   intake.intake();      // start collecting balls
//   intake.reverse();     // spit everything out
//   intake.stop();        // turn off
//   intake.shoot();       // feed balls into the flywheel (spin flywheel up first!)
//   intake.periodic();    // call EVERY loop, or nothing will move
//
// The constructor never throws. If a device named in the robot configuration
// is missing the intake simply reports itself unavailable (isAvailable() /
// getFault()) and every method becomes a no-op, so a typo in the config
// cannot stop the drive train from working.
// =====================================================================
public class Intake {
    // EJECT = full and an extra ball came in, so spit it back out (happens by itself)
    // SHOOT = open the door and run the rollers to push the balls into the flywheel
    public enum IntakeState { OFF, INTAKE, FULL, REVERSE, EJECT, SHOOT }

    // =====================================================================
    // HARDWARE NAMES - these are configuration, not tuning: they must match
    // the robot configuration on the Driver Hub exactly. A mismatch no longer
    // kills the OpMode; it shows up in getFault() instead.
    // =====================================================================
    private static final String LOWER_MOTOR_NAME = "lowerIntake";   // lower roller motor name
    private static final String UPPER_MOTOR_NAME = "upperIntake";   // upper roller motor name (delete if one motor drives both rollers)
    private static final String BEAM_BREAK_NAME  = "beamBreak";     // beam-break sensor name
    private static final String DOOR_SERVO_NAME  = "door";          // door servo name

    /** Servo positions closer together than this are treated as the same position. */
    private static final double DOOR_POSITION_EPSILON = 1e-3;

    // ---- hardware (null when the device is missing from the robot configuration) ----
    private DcMotorEx lowerRoller;
    private DcMotorEx upperRoller;   // delete this (and every upperRoller line) if one motor drives both rollers
    private DigitalChannel beamBreak;
    private Servo door;

    // ---- health ----
    private boolean available = false;
    private String fault = "not initialised";

    // ---- state ----
    private IntakeState state = IntakeState.OFF;
    private int ballCount = 0;
    private boolean ballWasInBeam = false;
    private final ElapsedTime countTimer = new ElapsedTime();
    private final ElapsedTime ejectTimer = new ElapsedTime();

    /** Last position written to the door servo; null until it has been written once. */
    private Double lastDoorPosition = null;

    public Intake(HardwareMap hardwareMap) {
        // Look each device up separately so the fault names the one that is actually wrong.
        StringBuilder problems = new StringBuilder();
        lowerRoller = lookup(hardwareMap, DcMotorEx.class, LOWER_MOTOR_NAME, problems);
        upperRoller = lookup(hardwareMap, DcMotorEx.class, UPPER_MOTOR_NAME, problems);
        beamBreak = lookup(hardwareMap, DigitalChannel.class, BEAM_BREAK_NAME, problems);
        door = lookup(hardwareMap, Servo.class, DOOR_SERVO_NAME, problems);

        if (problems.length() > 0) {
            // No configuration calls on a half-built intake: the OpMode keeps driving without it.
            available = false;
            fault = "robot configuration - " + problems;
            return;
        }

        // All four devices exist from here on; the configuration calls below may still fail.
        available = true;
        try {
            beamBreak.setMode(DigitalChannel.Mode.INPUT);

            lowerRoller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            upperRoller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lowerRoller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            upperRoller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            applyDirections();
            writeDoor(IntakeConfig.doorClosed);
            fault = "";
        } catch (RuntimeException e) {
            available = false;
            fault = "configuration - " + e.getClass().getSimpleName() + ": " + e.getMessage();
        }
    }

    private static <T> T lookup(HardwareMap hardwareMap, Class<T> type, String name,
                                StringBuilder problems) {
        try {
            return hardwareMap.get(type, name);
        } catch (RuntimeException e) {
            if (problems.length() > 0) problems.append("; ");
            problems.append('"').append(name).append("\" not found");
            return null;
        }
    }

    /** False when a device in the robot configuration is missing; every method is then a no-op. */
    public boolean isAvailable() {
        return available;
    }

    /** Why the intake is unavailable, in one line, for telemetry. Empty when it is healthy. */
    public String getFault() {
        return fault;
    }

    /** Pushes the direction flags in IntakeConfig down to the motors. */
    public void applyDirections() {
        if (!available) return;
        lowerRoller.setDirection(IntakeConfig.lowerReversed
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        upperRoller.setDirection(IntakeConfig.upperReversed
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }

    // =====================================================================
    // CONTROLS - call these from your OpMode (e.g. when a button is pressed)
    // =====================================================================
    public void intake()    { setState(IntakeState.INTAKE); }
    public void reverse()   { setState(IntakeState.REVERSE); }
    public void stop()      { setState(IntakeState.OFF); }
    public void shoot()     { setState(IntakeState.SHOOT); }

    public void setState(IntakeState newState) {
        // Shooting empties the intake, so start counting from 0 again. This is the ONLY thing that
        // lowers the count: EJECT throws away a ball that was never counted, so ballCount stays at
        // maxBalls and is only known to be right again after the next shoot().
        if (newState == IntakeState.SHOOT) {
            ballCount = 0;
        }
        state = newState;
    }

    public void resetBallCount() { ballCount = 0; }

    // ---- getters for telemetry and other mechanisms ----
    public IntakeState getState() { return state; }
    public int getBallCount() { return ballCount; }
    public boolean isFull() { return ballCount >= IntakeConfig.maxBalls; }
    public double getThrottle() { return available ? lowerRoller.getPower() : 0; }

    public String getSensorDebug() {
        if (!available) return "unavailable: " + fault;
        return "raw=" + beamBreak.getState() + " broken=" + isBeamBroken();
    }

    /** Raw sensor level, before the polarity in IntakeConfig is applied. */
    public boolean getRawBeamState() { return available && beamBreak.getState(); }

    /** Roller speeds in ticks/second; useful for spotting a stalled or unplugged encoder. */
    public double getLowerVelocity() { return available ? lowerRoller.getVelocity() : 0; }
    public double getUpperVelocity() { return available ? upperRoller.getVelocity() : 0; }

    // =====================================================================
    // TEST HOOKS - used by AutoTuner, not by match code
    // =====================================================================
    /** Runs the rollers directly, ignoring state and sensors. */
    public void setTestPower(double lower, double upper) {
        if (!available) return;
        lowerRoller.setPower(lower);
        upperRoller.setPower(upper);
    }

    public void stopMotors() {
        if (!available) return;
        lowerRoller.setPower(0);
        upperRoller.setPower(0);
    }

    // =====================================================================
    // call this once every loop from your OpMode
    // =====================================================================
    public void periodic() {
        if (!available) return;
        countBalls();
        updateAutoState();
        applyOutputs();
    }

    // =====================================================================
    // INSIDE STUFF - you shouldn't need to touch anything below here
    // =====================================================================

    // Most beam-break sensors read LOW (false) when something is blocking the beam, but the
    // polarity is an IntakeConfig flag so the AutoTuner can measure it instead of guessing.
    public boolean isBeamBroken() {
        if (!available) return false;
        boolean raw = beamBreak.getState();
        return IntakeConfig.beamActiveLow ? !raw : raw;
    }

    // 1. did a ball JUST arrive at the sensor? (wasn't there last loop, is there now)
    private void countBalls() {
        boolean ballInBeam = isBeamBroken();
        boolean ballArrived = ballInBeam
                && !ballWasInBeam
                && countTimer.seconds() > IntakeConfig.countCooldownSeconds;
        ballWasInBeam = ballInBeam;

        if (ballArrived && state == IntakeState.INTAKE && ballCount < IntakeConfig.maxBalls) {
            // room for it: count it
            ballCount++;
            countTimer.reset();
        } else if (ballArrived && state == IntakeState.FULL) {
            // already full: this is an extra ball, spit it out. It was never counted, so the
            // count is still right - but nothing here can lower it, which is why the number is
            // only trustworthy between a shoot() and the next eject.
            state = IntakeState.EJECT;
            ejectTimer.reset();
            countTimer.reset();
        }
    }

    // 2. switch automatically between INTAKE / FULL / EJECT
    private void updateAutoState() {
        // done spitting the extra ball out? go back to holding what we have
        if (state == IntakeState.EJECT && ejectTimer.seconds() > IntakeConfig.ejectTimeSeconds) {
            state = IntakeState.FULL;
        }
        if (state == IntakeState.INTAKE && ballCount >= IntakeConfig.maxBalls) {
            state = IntakeState.FULL;
        }
        if (state == IntakeState.FULL && ballCount < IntakeConfig.maxBalls) {
            state = IntakeState.INTAKE;
        }
    }

    // 3. set the motors and door to match the state
    private void applyOutputs() {
        double lowerPower;
        double upperPower;
        switch (state) {
            case INTAKE:    lowerPower = IntakeConfig.intakePower;  upperPower = IntakeConfig.intakePower;  break;
            case FULL:      lowerPower = IntakeConfig.fullPower;    upperPower = IntakeConfig.fullPower;    break;
            case REVERSE:   lowerPower = IntakeConfig.reversePower; upperPower = IntakeConfig.reversePower; break;
            // only the lower roller spits out the extra ball;
            // the upper roller keeps holding the balls we want to keep
            case EJECT:     lowerPower = IntakeConfig.ejectPower;   upperPower = IntakeConfig.fullPower;    break;
            case SHOOT:     lowerPower = IntakeConfig.shootPower;   upperPower = IntakeConfig.shootPower;   break;
            default:        lowerPower = 0.0;                   upperPower = 0.0;                   break;   // OFF
        }
        lowerRoller.setPower(lowerPower);
        upperRoller.setPower(upperPower);

        // door is only open while shooting; closed it keeps balls away from the flywheel
        writeDoor(state == IntakeState.SHOOT ? IntakeConfig.doorOpen : IntakeConfig.doorClosed);
    }

    /**
     * Writes the servo only when the requested position changes. The door has two states and
     * applyOutputs runs at 50 Hz, so an unconditional write is 50 identical I2C commands a second
     * in the same loop as the Pinpoint read.
     */
    private void writeDoor(double position) {
        if (!available) return;
        if (lastDoorPosition != null
                && Math.abs(lastDoorPosition - position) < DOOR_POSITION_EPSILON) {
            return;
        }
        lastDoorPosition = position;
        door.setPosition(position);
    }
}
