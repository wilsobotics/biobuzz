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
        // Exact inverse of BallProjector.project. World -> robot frame (fwd, left, up) by -camYaw.
        double yaw = m.camYaw(robot);
        double fwd = dx * Math.cos(yaw) + dy * Math.sin(yaw);
        double left = -dx * Math.sin(yaw) + dy * Math.cos(yaw);
        double up = ballH - m.heightIn;
        // Undo the pitch: fwd = zc cos p + yc sin p, up = -zc sin p + yc cos p.
        double p = Math.toRadians(m.pitchDownDeg);
        double zc = fwd * Math.cos(p) - up * Math.sin(p);
        double yc = fwd * Math.sin(p) + up * Math.cos(p);
        double xc = -left;
        if (zc <= 1e-9) {
            return false; // behind the image plane
        }
        double txDeg = Math.toDegrees(Math.atan(xc / zc));
        double tyDeg = Math.toDegrees(Math.atan(yc / zc));
        double margin = PerceptionConfig.EDGE_MARGIN_DEG;
        return Math.abs(txDeg) <= m.hfovDeg / 2 - margin && Math.abs(tyDeg) <= m.vfovDeg / 2 - margin;
    }
}
