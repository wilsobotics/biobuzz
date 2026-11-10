package org.firstinspires.ftc.teamcode.perception;

import java.util.List;

/** One detector (or colour) result in camera angles. aspect = box width / height (1 if unknown). */
public final class Detection {
    public final int classId;
    public final double confidence;
    public final double txDeg;
    public final double tyDeg;
    public final double aspect;

    public Detection(int classId, double confidence, double txDeg, double tyDeg, double aspect) {
        this.classId = classId;
        this.confidence = confidence;
        this.txDeg = txDeg;
        this.tyDeg = tyDeg;
        this.aspect = aspect;
    }

    public Detection withClassId(int id) {
        return new Detection(id, confidence, txDeg, tyDeg, aspect);
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
