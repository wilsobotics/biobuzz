package org.firstinspires.ftc.teamcode.perception;

import com.pedropathing.math.Pose;

/** Whether a floor point is inside the camera's usable view (FOV minus the edge margin, within maxRange). */
public final class CameraView {
    private CameraView() {
    }

    public static boolean canSee(Pose robot, CameraMount m, double px, double py, double ballH, double maxRange) {
        double cx = m.camX(robot);
        double cy = m.camY(robot);
        double dx = px - cx;
        double dy = py - cy;
        double r = Math.hypot(dx, dy);
        if (r > maxRange || r < 1e-6) {
            return false;
        }
        double bearingDeg = Math.toDegrees(Angles.wrap(Math.atan2(dy, dx) - m.camYaw(robot)));
        double txDeg = -bearingDeg;
        double depressionDeg = Math.toDegrees(Math.atan2(m.heightIn - ballH, r));
        double tyDeg = m.pitchDownDeg - depressionDeg;
        double margin = PerceptionConfig.EDGE_MARGIN_DEG;
        return Math.abs(txDeg) <= m.hfovDeg / 2 - margin && Math.abs(tyDeg) <= m.vfovDeg / 2 - margin;
    }
}
