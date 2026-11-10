package org.firstinspires.ftc.teamcode.perception;

import com.pedropathing.math.Pose;

/** A ball seen in one frame, in the field frame, with its position uncertainty and quality. */
public final class Sighting {
    public final BallClass ballClass;
    public final double x;
    public final double y;
    public final double range;
    public final double sigma;
    public final double quality;
    public final boolean maybeTwo;

    public Sighting(BallClass ballClass, double x, double y, double range, double sigma,
                    double quality, boolean maybeTwo) {
        this.ballClass = ballClass;
        this.x = x;
        this.y = y;
        this.range = range;
        this.sigma = sigma;
        this.quality = quality;
        this.maybeTwo = maybeTwo;
    }

    /** Projects a detection, or returns null for non-balls, junk confidence or rays that miss the floor. */
    public static Sighting from(Detection d, Pose robot, CameraMount m) {
        BallClass c = BallClass.fromClassId(d.classId);
        if (c == null || d.confidence < PerceptionConfig.MIN_CONFIDENCE) {
            return null;
        }
        double[] p = BallProjector.project(robot, m, d.txDeg, d.tyDeg, c.centerHeightIn());
        if (p == null) {
            return null;
        }
        double sigma = Math.max(PerceptionConfig.MIN_SIGMA_IN,
                BallProjector.errorPerDegree(p[2], m.heightIn, c.centerHeightIn()) * PerceptionConfig.ANGLE_SIGMA_DEG);
        return new Sighting(c, p[0], p[1], p[2], sigma,
                quality(d.confidence, p[2], d.txDeg, d.tyDeg, m),
                d.aspect > PerceptionConfig.MAYBE_TWO_ASPECT);
    }

    /** Confidence x range factor x frame-edge factor. */
    public static double quality(double conf, double range, double txDeg, double tyDeg, CameraMount m) {
        double good = PerceptionConfig.GOOD_RANGE_IN;
        double max = PerceptionConfig.MAX_RANGE_IN;
        double far = PerceptionConfig.FAR_RANGE_FACTOR;
        double rangeFactor;
        if (range <= good) {
            rangeFactor = 1.0;
        } else if (range >= max) {
            rangeFactor = far;
        } else {
            rangeFactor = 1.0 - (range - good) / (max - good) * (1.0 - far);
        }
        double margin = PerceptionConfig.EDGE_MARGIN_DEG;
        boolean edge = Math.abs(txDeg) > m.hfovDeg / 2 - margin || Math.abs(tyDeg) > m.vfovDeg / 2 - margin;
        return conf * rangeFactor * (edge ? PerceptionConfig.EDGE_FACTOR : 1.0);
    }
}
