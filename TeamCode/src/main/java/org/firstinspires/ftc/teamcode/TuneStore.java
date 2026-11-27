package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.content.SharedPreferences;

/**
 * Stores the AutoTuner's results on the Control Hub, so a match run never needs a tuning run:
 * TeleOpDriver, Intake and Localization load their saved values when they are constructed, and a
 * tuning run writes them back at the end.
 *
 * Everything lives in one SharedPreferences file, keyed by subsystem, so it can be inspected or
 * wiped from Android's app-data screen if a saved value ever needs to go away. Each load is
 * defensive: a missing key keeps the value compiled into the class, and a value outside the range
 * below is ignored rather than trusted, so a bad save cannot hand the driver a nonsense gain.
 *
 * SharedPreferences are floats, not doubles; that is plenty for these values.
 */
final class TuneStore {

    /** Bumping this name throws away every saved value (a "factory reset" of the tuning). */
    private static final String PREFS_NAME = "biobuzz_tuning_v1";

    // ---- drive (TeleOpDriver.Tuning) ----
    private static final String K_KP_TRANSLATIONAL = "drive.kpTranslational";
    private static final String K_KP_HEADING = "drive.kpHeading";
    private static final String K_MIN_HOLD_POWER = "drive.minHoldPower";
    private static final String K_HOLD_POSITION_DEADBAND = "drive.holdPositionDeadbandMm";
    private static final String K_HOLD_HEADING_DEADBAND = "drive.holdHeadingDeadbandRad";
    private static final String K_JOYSTICK_ENGAGE = "drive.joystickEngageDeadband";
    private static final String K_JOYSTICK_RELEASE = "drive.joystickReleaseDeadband";

    // ---- intake (Intake.Settings) ----
    private static final String K_BEAM_ACTIVE_LOW = "intake.beamActiveLow";
    private static final String K_LOWER_REVERSED = "intake.lowerReversed";
    private static final String K_UPPER_REVERSED = "intake.upperReversed";

    // ---- localisation (Localization.Config) ----
    private static final String K_X_POD_FORWARD = "localization.xPodForward";
    private static final String K_Y_POD_FORWARD = "localization.yPodForward";

    // Ranges a saved value has to fall inside to be believed. Anything else keeps the compiled
    // default, which is why these are wider than the values the tuner would ever produce.
    private static final double KP_TRANSLATIONAL_MIN = 0.0005, KP_TRANSLATIONAL_MAX = 0.0200;
    private static final double KP_HEADING_MIN = 0.20, KP_HEADING_MAX = 4.00;
    private static final double MIN_HOLD_POWER_MIN = 0.0, MIN_HOLD_POWER_MAX = 0.20;
    private static final double HOLD_POSITION_DEADBAND_MIN = 2.0, HOLD_POSITION_DEADBAND_MAX = 40.0;
    private static final double HOLD_HEADING_DEADBAND_MIN = 0.0, HOLD_HEADING_DEADBAND_MAX = Math.toRadians(8.0);
    private static final double JOYSTICK_ENGAGE_MIN = 0.02, JOYSTICK_ENGAGE_MAX = 0.30;
    private static final double JOYSTICK_RELEASE_MIN = 0.005, JOYSTICK_RELEASE_MAX = 0.30;

    private TuneStore() {
    }

    // ------------------------------------------------------------------
    // Drive
    // ------------------------------------------------------------------

    /** @return true if anything was loaded, i.e. the robot is running saved values. */
    static boolean loadTuning(Context context, TeleOpDriver.Tuning tuning) {
        SharedPreferences prefs = prefs(context);
        if (prefs == null) {
            return false;
        }
        tuning.kpTranslational = readDouble(prefs, K_KP_TRANSLATIONAL, tuning.kpTranslational,
                KP_TRANSLATIONAL_MIN, KP_TRANSLATIONAL_MAX);
        tuning.kpHeading = readDouble(prefs, K_KP_HEADING, tuning.kpHeading,
                KP_HEADING_MIN, KP_HEADING_MAX);
        tuning.minHoldPower = readDouble(prefs, K_MIN_HOLD_POWER, tuning.minHoldPower,
                MIN_HOLD_POWER_MIN, MIN_HOLD_POWER_MAX);
        tuning.holdPositionDeadbandMm = readDouble(prefs, K_HOLD_POSITION_DEADBAND,
                tuning.holdPositionDeadbandMm,
                HOLD_POSITION_DEADBAND_MIN, HOLD_POSITION_DEADBAND_MAX);
        tuning.holdHeadingDeadbandRad = readDouble(prefs, K_HOLD_HEADING_DEADBAND,
                tuning.holdHeadingDeadbandRad,
                HOLD_HEADING_DEADBAND_MIN, HOLD_HEADING_DEADBAND_MAX);
        tuning.joystickEngageDeadband = readDouble(prefs, K_JOYSTICK_ENGAGE,
                tuning.joystickEngageDeadband, JOYSTICK_ENGAGE_MIN, JOYSTICK_ENGAGE_MAX);
        tuning.joystickReleaseDeadband = readDouble(prefs, K_JOYSTICK_RELEASE,
                tuning.joystickReleaseDeadband, JOYSTICK_RELEASE_MIN, JOYSTICK_RELEASE_MAX);

        // A release band at or above the engage band would make the input latch stick on.
        if (tuning.joystickReleaseDeadband >= tuning.joystickEngageDeadband) {
            tuning.joystickReleaseDeadband = tuning.joystickEngageDeadband * 0.5;
        }
        return prefs.contains(K_KP_TRANSLATIONAL);
    }

    /** Saves the values the tuner may change. maxHoldPower is a safety cap and is never stored. */
    static boolean saveTuning(Context context, TeleOpDriver.Tuning tuning) {
        SharedPreferences prefs = prefs(context);
        if (prefs == null) {
            return false;
        }
        return prefs.edit()
                .putFloat(K_KP_TRANSLATIONAL, (float) tuning.kpTranslational)
                .putFloat(K_KP_HEADING, (float) tuning.kpHeading)
                .putFloat(K_MIN_HOLD_POWER, (float) tuning.minHoldPower)
                .putFloat(K_HOLD_POSITION_DEADBAND, (float) tuning.holdPositionDeadbandMm)
                .putFloat(K_HOLD_HEADING_DEADBAND, (float) tuning.holdHeadingDeadbandRad)
                .putFloat(K_JOYSTICK_ENGAGE, (float) tuning.joystickEngageDeadband)
                .putFloat(K_JOYSTICK_RELEASE, (float) tuning.joystickReleaseDeadband)
                .commit();
    }

    // ------------------------------------------------------------------
    // Intake
    // ------------------------------------------------------------------

    static boolean loadIntake(Context context, Intake.Settings settings) {
        SharedPreferences prefs = prefs(context);
        if (prefs == null) {
            return false;
        }
        settings.beamActiveLow = prefs.getBoolean(K_BEAM_ACTIVE_LOW, settings.beamActiveLow);
        settings.lowerReversed = prefs.getBoolean(K_LOWER_REVERSED, settings.lowerReversed);
        settings.upperReversed = prefs.getBoolean(K_UPPER_REVERSED, settings.upperReversed);
        return prefs.contains(K_BEAM_ACTIVE_LOW);
    }

    static boolean saveIntake(Context context, Intake.Settings settings) {
        SharedPreferences prefs = prefs(context);
        if (prefs == null) {
            return false;
        }
        return prefs.edit()
                .putBoolean(K_BEAM_ACTIVE_LOW, settings.beamActiveLow)
                .putBoolean(K_LOWER_REVERSED, settings.lowerReversed)
                .putBoolean(K_UPPER_REVERSED, settings.upperReversed)
                .commit();
    }

    // ------------------------------------------------------------------
    // Localisation
    // ------------------------------------------------------------------

    static boolean loadLocalization(Context context, Localization.Config config) {
        SharedPreferences prefs = prefs(context);
        if (prefs == null) {
            return false;
        }
        config.xPodForward = prefs.getBoolean(K_X_POD_FORWARD, config.xPodForward);
        config.yPodForward = prefs.getBoolean(K_Y_POD_FORWARD, config.yPodForward);
        return prefs.contains(K_X_POD_FORWARD);
    }

    /** Only the learned pod directions are stored; offsets are measured hardware, not tuning. */
    static boolean saveLocalization(Context context, Localization.Config config) {
        SharedPreferences prefs = prefs(context);
        if (prefs == null) {
            return false;
        }
        return prefs.edit()
                .putBoolean(K_X_POD_FORWARD, config.xPodForward)
                .putBoolean(K_Y_POD_FORWARD, config.yPodForward)
                .commit();
    }

    // ------------------------------------------------------------------
    // Plumbing
    // ------------------------------------------------------------------

    private static SharedPreferences prefs(Context context) {
        if (context == null) {
            return null;   // no app context: run on the compiled defaults
        }
        try {
            return context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE);
        } catch (RuntimeException e) {
            return null;   // storage unavailable: run on the compiled defaults
        }
    }

    private static double readDouble(SharedPreferences prefs, String key, double fallback,
                                     double min, double max) {
        if (!prefs.contains(key)) {
            return fallback;
        }
        double value = prefs.getFloat(key, (float) fallback);
        if (Double.isNaN(value) || value < min || value > max) {
            return fallback;
        }
        return value;
    }
}
