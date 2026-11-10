package org.firstinspires.ftc.teamcode.collect;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/** The slice of the Pedro follower the collect brain uses (faked in tests). */
public interface PathFollower {
    void follow(Path path);

    boolean isBusy();

    Pose pose();

    /** Closest point on the current path, or null when not following. */
    Pose closestPose();

    void stop();
}
