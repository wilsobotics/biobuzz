package org.firstinspires.ftc.teamcode.perception;

import com.pedropathing.math.Pose;

/** Current robot pose in the Pedro field frame. In production: {@code follower::pose}. */
public interface PoseSource {
    Pose pose();
}
