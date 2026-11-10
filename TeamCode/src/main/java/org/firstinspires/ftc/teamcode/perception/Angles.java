package org.firstinspires.ftc.teamcode.perception;

/** Angle helpers. Radians, CCW positive. */
public final class Angles {
    private Angles() {
    }

    /** Wraps an angle into (-pi, pi]. Non-finite input is returned unchanged. */
    public static double wrap(double rad) {
        if (Double.isNaN(rad) || Double.isInfinite(rad)) {
            return rad; // nothing sensible to wrap; never loop on it
        }
        double w = Math.atan2(Math.sin(rad), Math.cos(rad));
        return w <= -Math.PI ? Math.PI : w;
    }
}
