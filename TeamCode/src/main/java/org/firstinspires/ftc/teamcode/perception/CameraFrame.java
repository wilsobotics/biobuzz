package org.firstinspires.ftc.teamcode.perception;

import com.pedropathing.math.Pose;

import java.util.List;

/** A gated camera frame stamped with the robot pose at capture time. balls is empty in GOAL mode. */
public final class CameraFrame {
    public final long id;
    public final CameraMode mode;
    public final PipelineSpec pipeline;
    public final long captureTimeMs;
    public final Pose robotPose;
    public final List<Detection> balls;

    public CameraFrame(long id, CameraMode mode, PipelineSpec pipeline, long captureTimeMs,
                       Pose robotPose, List<Detection> balls) {
        this.id = id;
        this.mode = mode;
        this.pipeline = pipeline;
        this.captureTimeMs = captureTimeMs;
        this.robotPose = robotPose;
        this.balls = balls;
    }
}
