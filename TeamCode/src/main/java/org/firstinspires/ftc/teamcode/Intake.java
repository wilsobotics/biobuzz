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
// =====================================================================
public class Intake {
    // EJECT = full and an extra ball came in, so spit it back out (happens by itself)
    // SHOOT = open the door and run the rollers to push the balls into the flywheel
    public enum IntakeState { OFF, INTAKE, FULL, REVERSE, EJECT, SHOOT }

    /**
     * Every tunable value, with the values the robot ships with. The AutoTuner measures what it
     * can and leaves the rest for a human; paste the final numbers back into these defaults.
     */
    public static class Settings {
        // ---- motor powers (-1.0 to 1.0) ----
        public double intakePower = 0.8;      // speed while collecting balls
        public double fullPower = 0.2;        // slow hold once full (too high = balls jam/squish)
        public double reversePower = -0.8;    // speed when spitting everything out
        public double ejectPower = -0.8;      // lower roller backwards to spit out an extra ball
        public double shootPower = 1.0;       // lower it if balls hit the flywheel too fast to shoot well

        // ---- door servo positions (0.0 to 1.0) ----
        public double doorOpen = 0.5;         // position that lets balls into the flywheel
        public double doorClosed = 0.0;       // position that blocks the flywheel

        // ---- counting / timing ----
        public int maxBalls = 4;              // how many balls the robot can hold
        public double countCooldownSeconds = 0.2;   // raise if one ball counts twice (holes in the balls)
        public double ejectTimeSeconds = 0.5;       // long enough for the extra ball to fully leave

        // ---- hardware behaviour ----
        public boolean beamActiveLow = true;  // most beam-break sensors read LOW when blocked
        public boolean lowerReversed = true;  // flips if the lower roller spins the wrong way
        public boolean upperReversed = false; // flips if the upper roller spins the wrong way
    }

    // =====================================================================
    // HARDWARE NAMES - these are configuration, not tuning: they must match
    // the robot configuration on the Driver Hub exactly.
    // =====================================================================
    private static final String LOWER_MOTOR_NAME = "lowerIntake";   // lower roller motor name
    private static final String UPPER_MOTOR_NAME = "upperIntake";   // upper roller motor name (delete if one motor drives both rollers)
    private static final String BEAM_BREAK_NAME  = "beamBreak";     // beam-break sensor name
    private static final String DOOR_SERVO_NAME  = "door";          // door servo name

    private final Settings settings;

    // ---- hardware ----
    private final DcMotorEx lowerRoller;
    private final DcMotorEx upperRoller;   // delete this (and every upperRoller line) if one motor drives both rollers
    private final DigitalChannel beamBreak;
    private final Servo door;

    // ---- state ----
    private IntakeState state = IntakeState.OFF;
    private int ballCount = 0;
    private boolean ballWasInBeam = false;
    private final ElapsedTime countTimer = new ElapsedTime();
    private final ElapsedTime ejectTimer = new ElapsedTime();

    public Intake(HardwareMap hardwareMap) {
        this(hardwareMap, new Settings());
    }

    public Intake(HardwareMap hardwareMap, Settings settings) {
        this.settings = settings;

        lowerRoller = hardwareMap.get(DcMotorEx.class, LOWER_MOTOR_NAME);
        upperRoller = hardwareMap.get(DcMotorEx.class, UPPER_MOTOR_NAME);
        beamBreak = hardwareMap.get(DigitalChannel.class, BEAM_BREAK_NAME);
        beamBreak.setMode(DigitalChannel.Mode.INPUT);
        door = hardwareMap.get(Servo.class, DOOR_SERVO_NAME);

        lowerRoller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        upperRoller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lowerRoller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        upperRoller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        applyDirections();
        door.setPosition(settings.doorClosed);
    }

    public Settings getSettings() {
        return settings;
    }

    /** Pushes the direction flags in Settings down to the motors. */
    public void applyDirections() {
        lowerRoller.setDirection(settings.lowerReversed
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        upperRoller.setDirection(settings.upperReversed
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
        // shooting empties the intake, so start counting from 0 again
        if (newState == IntakeState.SHOOT) {
            ballCount = 0;
        }
        state = newState;
    }

    public void resetBallCount() { ballCount = 0; }

    // ---- getters for telemetry and other mechanisms ----
    public IntakeState getState() { return state; }
    public int getBallCount() { return ballCount; }
    public boolean isFull() { return ballCount >= settings.maxBalls; }
    public double getThrottle() { return lowerRoller.getPower(); }
    public String getSensorDebug() { return "beam raw=" + beamBreak.getState() + " broken=" + isBeamBroken(); }

    /** Raw sensor level, before the polarity in Settings is applied. */
    public boolean getRawBeamState() { return beamBreak.getState(); }

    /** Roller speeds in ticks/second; useful for spotting a stalled or unplugged encoder. */
    public double getLowerVelocity() { return lowerRoller.getVelocity(); }
    public double getUpperVelocity() { return upperRoller.getVelocity(); }

    // =====================================================================
    // TEST HOOKS - used by AutoTuner, not by match code
    // =====================================================================
    /** Runs the rollers directly, ignoring state and sensors. */
    public void setTestPower(double lower, double upper) {
        lowerRoller.setPower(lower);
        upperRoller.setPower(upper);
    }

    public void stopMotors() {
        lowerRoller.setPower(0);
        upperRoller.setPower(0);
    }

    // =====================================================================
    // call this once every loop from your OpMode
    // =====================================================================
    public void periodic() {
        countBalls();
        updateAutoState();
        applyOutputs();
    }

    // =====================================================================
    // INSIDE STUFF - you shouldn't need to touch anything below here
    // =====================================================================

    // Most beam-break sensors read LOW (false) when something is blocking the beam, but the
    // polarity is a Settings flag so the AutoTuner can measure it instead of guessing.
    public boolean isBeamBroken() {
        boolean raw = beamBreak.getState();
        return settings.beamActiveLow ? !raw : raw;
    }

    // 1. did a ball JUST arrive at the sensor? (wasn't there last loop, is there now)
    private void countBalls() {
        boolean ballInBeam = isBeamBroken();
        boolean ballArrived = ballInBeam
                && !ballWasInBeam
                && countTimer.seconds() > settings.countCooldownSeconds;
        ballWasInBeam = ballInBeam;

        if (ballArrived && state == IntakeState.INTAKE && ballCount < settings.maxBalls) {
            // room for it: count it
            ballCount++;
            countTimer.reset();
        } else if (ballArrived && state == IntakeState.FULL) {
            // already full: this is an extra ball, spit it out
            state = IntakeState.EJECT;
            ejectTimer.reset();
            countTimer.reset();
        }
    }

    // 2. switch automatically between INTAKE / FULL / EJECT
    private void updateAutoState() {
        // done spitting the extra ball out? go back to holding what we have
        if (state == IntakeState.EJECT && ejectTimer.seconds() > settings.ejectTimeSeconds) {
            state = IntakeState.FULL;
        }
        if (state == IntakeState.INTAKE && ballCount >= settings.maxBalls) {
            state = IntakeState.FULL;
        }
        if (state == IntakeState.FULL && ballCount < settings.maxBalls) {
            state = IntakeState.INTAKE;
        }
    }

    // 3. set the motors and door to match the state
    private void applyOutputs() {
        double lowerPower;
        double upperPower;
        switch (state) {
            case INTAKE:    lowerPower = settings.intakePower;  upperPower = settings.intakePower;  break;
            case FULL:      lowerPower = settings.fullPower;    upperPower = settings.fullPower;    break;
            case REVERSE:   lowerPower = settings.reversePower; upperPower = settings.reversePower; break;
            // only the lower roller spits out the extra ball;
            // the upper roller keeps holding the balls we want to keep
            case EJECT:     lowerPower = settings.ejectPower;   upperPower = settings.fullPower;    break;
            case SHOOT:     lowerPower = settings.shootPower;   upperPower = settings.shootPower;   break;
            default:        lowerPower = 0.0;                   upperPower = 0.0;                   break;   // OFF
        }
        lowerRoller.setPower(lowerPower);
        upperRoller.setPower(upperPower);

        // door is only open while shooting; closed it keeps balls away from the flywheel
        door.setPosition(state == IntakeState.SHOOT ? settings.doorOpen : settings.doorClosed);
    }
}
