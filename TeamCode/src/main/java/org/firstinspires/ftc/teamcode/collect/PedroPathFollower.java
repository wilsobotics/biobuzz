package org.firstinspires.ftc.teamcode.collect;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/** PathFollower over a Pedro 3 Follower. Whoever owns the follower must call follower.update() every loop. */
public final class PedroPathFollower implements PathFollower {
    private final Follower follower;

    public PedroPathFollower(Follower follower) {
        this.follower = follower;
    }

    @Override public void follow(Path path) { follower.follow(path); }
    @Override public boolean isBusy() { return follower.isBusy(); }
    @Override public Pose pose() { return follower.pose(); }
    @Override public Pose closestPose() { return follower.following() ? follower.closestPose() : null; }
    @Override public void stop() { follower.stop(); }
}
