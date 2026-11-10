package org.firstinspires.ftc.teamcode.perception;

import com.pedropathing.math.Pose;

/** Limelight angles -> field position. Port of Hive Vision BallMath.camRay/ballFieldPos. */
public final class BallProjector {
    private BallProjector() {
    }

    /**
     * Field position of a ball centre seen at (tx, ty).
     *
     * @return {x, y, horizontal range from the camera} in inches, or null if the ray never comes down to ballCenterH
     */
    public static double[] project(Pose robot, CameraMount m, double txDeg, double tyDeg, double ballCenterH) {
        double tx = Math.toRadians(txDeg);
        double ty = Math.toRadians(tyDeg);
        double p = Math.toRadians(m.pitchDownDeg);
        double txn = Math.tan(tx);
        double tyn = Math.tan(ty);
        double zc = 1.0 / Math.sqrt(1.0 + txn * txn + tyn * tyn);
        double xc = txn * zc;
        double yc = tyn * zc;
        double fwd = zc * Math.cos(p) + yc * Math.sin(p);
        double left = -xc;
        double up = -zc * Math.sin(p) + yc * Math.cos(p);
        double dz = ballCenterH - m.heightIn;
        if (up >= -1e-9 || dz >= 0) {
            return null;
        }
        double t = dz / up;
        double yaw = m.camYaw(robot);
        double wx = fwd * Math.cos(yaw) - left * Math.sin(yaw);
        double wy = fwd * Math.sin(yaw) + left * Math.cos(yaw);
        double cx = m.camX(robot);
        double cy = m.camY(robot);
        double bx = cx + t * wx;
        double by = cy + t * wy;
        return new double[]{bx, by, Math.hypot(bx - cx, by - cy)};
    }

    /** Floor position error (in) per degree of vertical angle error at a horizontal range. */
    public static double errorPerDegree(double range, double camH, double ballH) {
        double dz = camH - ballH;
        return (range * range + dz * dz) / dz * Math.PI / 180.0;
    }
}
