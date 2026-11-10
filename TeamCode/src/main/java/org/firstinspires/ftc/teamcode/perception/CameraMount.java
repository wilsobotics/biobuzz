package org.firstinspires.ftc.teamcode.perception;

import com.pedropathing.math.Pose;

/** Where the ball camera sits on the robot. Pitch is degrees BELOW horizontal; yaw is CCW from robot forward. */
public final class CameraMount {
    public final double forwardIn;
    public final double leftIn;
    public final double heightIn;
    public final double pitchDownDeg;
    public final double yawDeg;
    public final double hfovDeg;
    public final double vfovDeg;

    public CameraMount(double forwardIn, double leftIn, double heightIn, double pitchDownDeg,
                       double yawDeg, double hfovDeg, double vfovDeg) {
        this.forwardIn = forwardIn;
        this.leftIn = leftIn;
        this.heightIn = heightIn;
        this.pitchDownDeg = pitchDownDeg;
        this.yawDeg = yawDeg;
        this.hfovDeg = hfovDeg;
        this.vfovDeg = vfovDeg;
    }

    /** The ball-mode mount from PerceptionConfig. */
    public static CameraMount fromConfig() {
        return new CameraMount(PerceptionConfig.CAM_FORWARD_IN, PerceptionConfig.CAM_LEFT_IN,
                PerceptionConfig.CAM_HEIGHT_IN, PerceptionConfig.BALL_PITCH_DOWN_DEG,
                PerceptionConfig.TURRET_BALL_YAW_DEG, PerceptionConfig.HFOV_DEG, PerceptionConfig.VFOV_DEG);
    }

    public double camX(Pose robot) {
        double h = robot.heading();
        return robot.x() + forwardIn * Math.cos(h) - leftIn * Math.sin(h);
    }

    public double camY(Pose robot) {
        double h = robot.heading();
        return robot.y() + forwardIn * Math.sin(h) + leftIn * Math.cos(h);
    }

    public double camYaw(Pose robot) {
        return robot.heading() + Math.toRadians(yawDeg);
    }
}
