package org.firstinspires.ftc.teamcode.collect;

import org.firstinspires.ftc.teamcode.perception.Angles;

/**
 * Converts the manual frame (x along the audience side A->F, y away from the audience) into
 * the Pedro field frame: a rotation by quarter turns (CCW) about the field centre.
 */
public final class FieldFrame {
    private final int quarterTurns;

    public FieldFrame(double rotationDeg) {
        this.quarterTurns = (((int) Math.round(rotationDeg / 90.0)) % 4 + 4) % 4;
    }

    public static FieldFrame fromConfig() {
        return new FieldFrame(CollectConfig.PEDRO_ROTATION_DEG);
    }

    public double[] toPedro(double x, double y) {
        return rotate(x, y, quarterTurns);
    }

    public double[] toManual(double x, double y) {
        return rotate(x, y, (4 - quarterTurns) % 4);
    }

    public double headingToPedro(double rad) {
        return Angles.wrap(rad + quarterTurns * Math.PI / 2);
    }

    private static double[] rotate(double x, double y, int q) {
        double dx = x - 72;
        double dy = y - 72;
        for (int i = 0; i < q; i++) {
            double t = dx;
            dx = -dy;
            dy = t;
        }
        return new double[]{72 + dx, 72 + dy};
    }
}
