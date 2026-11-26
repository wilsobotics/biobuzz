package org.firstinspires.ftc.teamcode;

<<<<<<< Updated upstream
=======
import android.content.Context;

>>>>>>> Stashed changes
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
<<<<<<< Updated upstream

=======
>>>>>>> Stashed changes
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

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
<<<<<<< Updated upstream
    public enum IntakeState { OFF, INTAKE, FULL, REVERSE, EJECT, SHOOT, SHOOTPIDF}

    // =====================================================================
    // SETTINGS - everything marked "TODO" below is a placeholder.
    //   TODO REPLACE = must change before the robot will run
    //   TODO TUNE    = works as a starting guess, adjust while testing
    // Tip: View > Tool Windows > TODO lists every one of these.
    // =====================================================================

    // ---- hardware names: must EXACTLY match the robot configuration on the Driver Hub ----
    private static final String LOWER_MOTOR_NAME = "lowerIntake";   // TODO REPLACE: lower roller motor name
    private static final String UPPER_MOTOR_NAME = "upperIntake";   // TODO REPLACE: upper roller motor name (delete if one motor drives both rollers)
    private static final String BEAM_BREAK_NAME  = "beamBreak";     // TODO REPLACE: beam-break sensor name
    private static final String DOOR_SERVO_NAME  = "door";          // TODO REPLACE: door servo name

    // ---- door servo positions (0.0 to 1.0) ----
    private static final double DOOR_OPEN   = 0.5;   // TODO REPLACE: position that lets balls into the flywheel
    private static final double DOOR_CLOSED = 0.0;   // TODO REPLACE: position that blocks the flywheel

    // ---- motor powers (-1.0 to 1.0) ----
    private static final double INTAKE_POWER  = 0.8;    // TODO TUNE: speed while collecting balls
    private static final double FULL_POWER    = 0.2;    // TODO TUNE: slow hold once full (too high = balls jam/squish)
    private static final double REVERSE_POWER = -0.8;   // TODO TUNE: speed when spitting everything out
    private static final double EJECT_POWER   = -0.8;   // TODO TUNE: lower roller backwards to spit out an extra ball
    private static final double SHOOT_POWER   = 1.0;    // TODO TUNE: lower it if balls hit the flywheel too fast to shoot well
    private static final double UPPER_TARGET_V = 1000;   // TODO TUNE: see whats the best v
    // ---- counting / timing ----
    private static final int MAX_BALLS = 4;                     // TODO REPLACE: how many balls the robot can hold
    private static final double COUNT_COOLDOWN_SECONDS = 0.2;   // TODO TUNE: raise if one ball counts twice (holes in the balls)
    private static final double EJECT_TIME_SECONDS = 0.5;       // TODO TUNE: long enough for the extra ball to fully leave

    // ---- backup full check: motor current (balls pile up -> roller strains -> current goes up) ----
    private static final double FULL_CURRENT_AMPS = 3.0;        // TODO TUNE: watch getCurrentDebug() with 3 vs 4 balls, pick a value between
    private static final double FULL_CURRENT_SECONDS = 0.25;    // TODO TUNE: how long it must stay high (ignores quick spikes when the motor starts)

    // ---- hardware ----
    private final DcMotorEx lowerRoller;
    private final DcMotorEx upperRoller;   // TODO REPLACE: delete this (and every upperRoller line) if one motor drives both rollers
    private final DigitalChannel beamBreak;
    private final Servo door;
    private PIDf transferPIDf;   // made fresh each time shootPIDF() is called
=======
    public enum IntakeState { OFF, INTAKE, FULL, REVERSE, EJECT, SHOOT, SHOOTPIDF }

    /**
     * Every tunable value, with the values the robot ships with. The AutoTuner measures what it
     * can and leaves the rest for a human: the AutoTuner edits the live instance returned by
     * getSettings() in place, so these literals are only the fallback for a fresh run.
     */
    public static class Settings {
        // ---- motor powers (-1.0 to 1.0) ----
        public double intakePower = 0.8;      // speed while collecting balls
        public double fullPower = 0.2;        // slow hold once full (too high = balls jam/squish)
        public double reversePower = -0.8;    // speed when spitting everything out
        public double ejectPower = -0.8;      // lower roller backwards to spit out an extra ball
        public double shootPower = 1.0;       // lower it if balls hit the flywheel too fast to shoot well

        public double upper_target_v = 1000;

        // ---- door servo positions (0.0 to 1.0) ----
        public double doorOpen = 0.5;         // position that lets balls into the flywheel
        public double doorClosed = 0.0;       // position that blocks the flywheel

        // ---- counting / timing ----
        public int maxBalls = 4;              // how many balls the robot can hold
        public double countCooldownSeconds = 0.2;   // raise if one ball counts twice (holes in the balls)
        public double ejectTimeSeconds = 0.5;       // long enough for the extra ball to fully leave

        public double full_current_amps = 3.0;

        public double full_current_seconds = 0.25;

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
    private final Context appContext;
    private boolean settingsLoaded = false;

    // ---- hardware ----
    private final DcMotorEx lowerRoller;
    private final DcMotorEx upperRoller;   // delete this (and every upperRoller line) if one motor drives both rollers
    private final DigitalChannel beamBreak;
    private final Servo door;
    private PIDf transferPIDF;
>>>>>>> Stashed changes

    // ---- state ----
    private IntakeState state = IntakeState.OFF;
    private int ballCount = 0;
    private boolean ballWasInBeam = false;
    private final ElapsedTime countTimer = new ElapsedTime();
    private final ElapsedTime ejectTimer = new ElapsedTime();
    private final ElapsedTime overCurrentTimer = new ElapsedTime();

    public Intake(HardwareMap hardwareMap) {
<<<<<<< Updated upstream
=======
        this(hardwareMap, new Settings());
    }

    public Intake(HardwareMap hardwareMap, Settings settings) {
        this.settings = settings;
        this.appContext = hardwareMap.appContext;
        // Saved values from the last AutoTuner run, if there are any.
        this.settingsLoaded = TuneStore.loadIntake(appContext, settings);

>>>>>>> Stashed changes
        lowerRoller = hardwareMap.get(DcMotorEx.class, LOWER_MOTOR_NAME);
        upperRoller = hardwareMap.get(DcMotorEx.class, UPPER_MOTOR_NAME);
        beamBreak = hardwareMap.get(DigitalChannel.class, BEAM_BREAK_NAME);
        beamBreak.setMode(DigitalChannel.Mode.INPUT);
        door = hardwareMap.get(Servo.class, DOOR_SERVO_NAME);
<<<<<<< Updated upstream
        door.setPosition(DOOR_CLOSED);

        lowerRoller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        upperRoller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // TODO TEST: if the LOWER roller is the one that strains when full, use lowerRoller here and in currentSaysFull()
        upperRoller.setCurrentAlert(FULL_CURRENT_AMPS, CurrentUnit.AMPS);
        // TODO TEST on the robot: run INTAKE and watch the rollers. Both must push the
        // ball INWARD. If one spins the wrong way, flip its direction here.
        lowerRoller.setDirection(DcMotorSimple.Direction.REVERSE);
        upperRoller.setDirection(DcMotorSimple.Direction.FORWARD);
=======

        lowerRoller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        upperRoller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lowerRoller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        upperRoller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        upperRoller.setCurrentAlert(settings.full_current_amps, CurrentUnit.AMPS);

        applyDirections();
        door.setPosition(settings.doorClosed);
    }

    public Settings getSettings() {
        return settings;
    }

    /** Persists the direction / beam flags the AutoTuner can change. */
    public boolean saveSettings() {
        return TuneStore.saveIntake(appContext, settings);
    }

    /** True if the constructor loaded saved settings instead of the compiled defaults. */
    public boolean isSettingsLoadedFromStore() {
        return settingsLoaded;
    }

    /** Pushes the direction flags in Settings down to the motors. */
    public void applyDirections() {
        lowerRoller.setDirection(settings.lowerReversed
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        upperRoller.setDirection(settings.upperReversed
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
>>>>>>> Stashed changes
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
        if (newState == IntakeState.SHOOT | newState == IntakeState.SHOOTPIDF) {
            ballCount = 0;
        }
        if (newState == IntakeState.SHOOTPIDF) {
<<<<<<< Updated upstream
            transferPIDf = new PIDf(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        }
        this.state = newState;
    }
    public void shootPIDF() {
        setState(IntakeState.SHOOTPIDF);
    }
    public void resetBallCount() { this.ballCount = 0; }
=======
            transferPIDF = new PIDf(0.0,0.0,0.0,0.0,0.0,0.0);
        }
        this.state = newState;
    }

    public void shootPIDf() {
        setState(IntakeState.SHOOTPIDF);
    }
    public void resetBallCount() {
        this.ballCount = 0;
    }

>>>>>>> Stashed changes

    // ---- getters for telemetry and other mechanisms ----
    public IntakeState getState() { return state; }
    public int getBallCount() { return ballCount; }
<<<<<<< Updated upstream
    public boolean isFull() { return ballCount >= MAX_BALLS; }
    public double getThrottle() { return lowerRoller.getPower(); }
    public String getSensorDebug() { return "beam raw=" + beamBreak.getState() + " broken=" + isBeamBroken(); }
    public String getCurrentDebug() { return "upper amps=" + upperRoller.getCurrent(CurrentUnit.AMPS) + " over=" + upperRoller.isOverCurrent(); }
=======
    public boolean isFull() { return ballCount >= settings.maxBalls; }
    public double getThrottle() { return lowerRoller.getPower(); }
    public String getSensorDebug() { return "beam raw=" + beamBreak.getState() + " broken=" + isBeamBroken(); }

    public String getCurrentDebug() {
        return "upper amps=" + upperRoller.getCurrent(CurrentUnit.AMPS) + " over=" + upperRoller.isOverCurrent();
    }
    /** Raw sensor level, before the polarity in Settings is applied. */
    public boolean getRawBeamState() { return beamBreak.getState(); }

    /** Roller speeds in ticks/second; useful for spotting a stalled or unplugged encoder. */
    public double getLowerVelocity() { return lowerRoller.getVelocity(); }
    public double getUpperVelocity() { return upperRoller.getVelocity(); }

    private boolean currentSaysFull() {
        if (!upperRoller.isOverCurrent()) {
            overCurrentTimer.reset();
        }
        return overCurrentTimer.seconds() > settings.full_current_seconds;
    }

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
>>>>>>> Stashed changes

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

<<<<<<< Updated upstream
    // Most beam-break sensors read LOW (false) when something is blocking the beam.
    // TODO TEST on the robot: show getSensorDebug() in telemetry. "broken" should be true
    // ONLY when a ball is in the beam. If it's backwards, remove the "!" below.
    private boolean isBeamBroken() {
        return !beamBreak.getState();
=======
    // Most beam-break sensors read LOW (false) when something is blocking the beam, but the
    // polarity is a Settings flag so the AutoTuner can measure it instead of guessing.
    public boolean isBeamBroken() {
        boolean raw = beamBreak.getState();
        return settings.beamActiveLow ? !raw : raw;
>>>>>>> Stashed changes
    }

    // 1. did a ball JUST arrive at the sensor? (wasn't there last loop, is there now)
    private void countBalls() {
        boolean ballInBeam = isBeamBroken();
        boolean ballArrived = ballInBeam
                && !ballWasInBeam
<<<<<<< Updated upstream
                && countTimer.seconds() > COUNT_COOLDOWN_SECONDS;
        ballWasInBeam = ballInBeam;

        if (ballArrived && state == IntakeState.INTAKE && ballCount < MAX_BALLS) {
=======
                && countTimer.seconds() > settings.countCooldownSeconds;
        ballWasInBeam = ballInBeam;

        if (ballArrived && state == IntakeState.INTAKE && ballCount < settings.maxBalls) {
>>>>>>> Stashed changes
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

<<<<<<< Updated upstream
    // backup check: has the roller been straining (over the current limit) for a bit?
    private boolean currentSaysFull() {
        if (!upperRoller.isOverCurrent()) {
            overCurrentTimer.reset();
        }
        return overCurrentTimer.seconds() > FULL_CURRENT_SECONDS;
    }

    // 2. switch automatically between INTAKE / FULL / EJECT
    private void updateAutoState() {
        // backup: if the beam break missed a ball but the roller is straining, we're full
        boolean strainingFull = currentSaysFull();
        if (state == IntakeState.INTAKE && strainingFull) {
            ballCount = MAX_BALLS;
        }
        // done spitting the extra ball out? go back to holding what we have
        if (state == IntakeState.EJECT && ejectTimer.seconds() > EJECT_TIME_SECONDS) {
            state = IntakeState.FULL;
        }
        if (state == IntakeState.INTAKE && ballCount >= MAX_BALLS) {
            state = IntakeState.FULL;
        }
        if (state == IntakeState.FULL && ballCount < MAX_BALLS) {
=======
    // 2. switch automatically between INTAKE / FULL / EJECT
    private void updateAutoState() {
        boolean strainingFull = currentSaysFull();
        if (state == IntakeState.INTAKE && strainingFull) {
            ballCount = settings.maxBalls;
        }

        // done spitting the extra ball out? go back to holding what we have
        if (state == IntakeState.EJECT && ejectTimer.seconds() > settings.ejectTimeSeconds) {
            state = IntakeState.FULL;
        }
        if (state == IntakeState.INTAKE && ballCount >= settings.maxBalls) {
            state = IntakeState.FULL;
        }
        if (state == IntakeState.FULL && ballCount < settings.maxBalls) {
>>>>>>> Stashed changes
            state = IntakeState.INTAKE;
        }
    }

    // 3. set the motors and door to match the state
    private void applyOutputs() {
        double lowerPower;
        double upperPower;
        switch (state) {
<<<<<<< Updated upstream
            case INTAKE:    lowerPower = INTAKE_POWER;  upperPower = INTAKE_POWER;  break;
            case FULL:      lowerPower = FULL_POWER;    upperPower = FULL_POWER;    break;
            case REVERSE:   lowerPower = REVERSE_POWER; upperPower = REVERSE_POWER; break;
            // only the lower roller spits out the extra ball;
            // the upper roller keeps holding the balls we want to keep
            case EJECT:     lowerPower = EJECT_POWER;   upperPower = FULL_POWER;    break;
            case SHOOT:     lowerPower = SHOOT_POWER;   upperPower = SHOOT_POWER;   break;
            case SHOOTPIDF: lowerPower = SHOOT_POWER; upperPower = transferPIDf.output(upperRoller.getVelocity(), UPPER_TARGET_V); break;
            default:        lowerPower = 0.0;           upperPower = 0.0;           break;   // OFF
=======
            case INTAKE:    lowerPower = settings.intakePower;  upperPower = settings.intakePower;  break;
            case FULL:      lowerPower = settings.fullPower;    upperPower = settings.fullPower;    break;
            case REVERSE:   lowerPower = settings.reversePower; upperPower = settings.reversePower; break;
            // only the lower roller spits out the extra ball;
            // the upper roller keeps holding the balls we want to keep
            case EJECT:     lowerPower = settings.ejectPower;   upperPower = settings.fullPower;    break;
            case SHOOT:     lowerPower = settings.shootPower;   upperPower = settings.shootPower;   break;
            case SHOOTPIDF: lowerPower = settings.shootPower; upperPower = transferPIDF.output(upperRoller.getVelocity(), settings.upper_target_v); break;
            default:        lowerPower = 0.0;                   upperPower = 0.0;                   break;   // OFF
>>>>>>> Stashed changes
        }
        lowerRoller.setPower(lowerPower);
        upperRoller.setPower(upperPower);

        // door is only open while shooting; closed it keeps balls away from the flywheel
<<<<<<< Updated upstream
        door.setPosition(state == IntakeState.SHOOT || state == IntakeState.SHOOTPIDF ? DOOR_OPEN : DOOR_CLOSED);
=======
        door.setPosition(state == IntakeState.SHOOT ? settings.doorOpen : settings.doorClosed);
>>>>>>> Stashed changes
    }
}
