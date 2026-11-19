package org.firstinspires.ftc.teamcode.perception;

import java.util.List;

/**
 * One detector (or colour) result in camera angles. aspect = box width / height (1 if unknown);
 * areaPct = Limelight ta, the box's share of the image in percent (0 if unknown).
 */
public final class Detection {
    public final int classId;
    public final double confidence;
    public final double txDeg;
    public final double tyDeg;
    public final double aspect;
    public final double areaPct;

    public Detection(int classId, double confidence, double txDeg, double tyDeg, double aspect) {
        this(classId, confidence, txDeg, tyDeg, aspect, 0.0);
    }

    public Detection(int classId, double confidence, double txDeg, double tyDeg, double aspect, double areaPct) {
        this.classId = classId;
        this.confidence = confidence;
        this.txDeg = txDeg;
        this.tyDeg = tyDeg;
        this.aspect = aspect;
        this.areaPct = areaPct;
    }

    public Detection withClassId(int id) {
        return new Detection(id, confidence, txDeg, tyDeg, aspect, areaPct);
    }

    /**
     * The box's smaller side in degrees (a ball's diameter, even when two balls merge into one
     * wide box), from ta and the field of view; 0 if the size is unknown.
     */
    public double sizeDeg(double hfovDeg, double vfovDeg) {
        if (areaPct <= 0 || aspect <= 0) {
            return 0.0;
        }
        double areaDeg2 = areaPct / 100.0 * hfovDeg * vfovDeg;
        double w = Math.sqrt(areaDeg2 * aspect);
        double h = Math.sqrt(areaDeg2 / aspect);
        return Math.min(w, h);
    }

    /** Width/height of the box spanned by Limelight target corners ([[x, y], ...] in pixels). */
    public static double aspectFromCorners(List<List<Double>> corners) {
        if (corners == null || corners.isEmpty()) {
            return 1.0;
        }
        double minX = Double.MAX_VALUE;
        double maxX = -Double.MAX_VALUE;
        double minY = Double.MAX_VALUE;
        double maxY = -Double.MAX_VALUE;
        for (List<Double> c : corners) {
            if (c == null || c.size() < 2) {
                continue;
            }
            minX = Math.min(minX, c.get(0));
            maxX = Math.max(maxX, c.get(0));
            minY = Math.min(minY, c.get(1));
            maxY = Math.max(maxY, c.get(1));
        }
        double h = maxY - minY;
        double w = maxX - minX;
        return h > 1e-9 && w > 0 ? w / h : 1.0;
    }
}
