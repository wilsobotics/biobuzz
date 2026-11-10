package org.firstinspires.ftc.teamcode.collect;

import com.pedropathing.math.Pose;

/** Event-driven re-plan decisions with a cooldown (spec Q7). */
public final class ReplanPolicy {
    public enum Reason {
        NONE, POSE_JUMP, PATH_DONE, TARGET_GONE, UNEXPECTED_PICKUP, MISSED_TARGET, OFF_PATH, STALLED, BETTER_BALL
    }

    public static final class Inputs {
        public boolean pathDone;
        public boolean targetGone;
        public boolean betterBall;
        public boolean unexpectedPickup;
        public boolean missedTarget;
        public boolean poseJump;
        public boolean following;
        public double offPathIn;
        public double headingErrDeg;
        public Pose pose;
    }

    private long lastReplanMs;
    private long lastPoseJumpMs = Long.MIN_VALUE / 2;
    private long stallSinceMs = -1;
    private long headingErrSinceMs = -1;
    private double anchorX;
    private double anchorY;

    /** Call when a re-plan happened outside check() (e.g. at start). */
    public void reset(long nowMs) {
        lastReplanMs = nowMs;
        stallSinceMs = -1;
        headingErrSinceMs = -1;
    }

    public Reason check(long nowMs, Inputs in) {
        Reason r = decide(nowMs, in);
        if (r != Reason.NONE) {
            if (r == Reason.POSE_JUMP) {
                lastPoseJumpMs = nowMs;
            }
            reset(nowMs);
        }
        return r;
    }

    private Reason decide(long nowMs, Inputs in) {
        // POSE_JUMP and PATH_DONE skip the general cooldown but have their own minimum intervals,
        // so a relocalisation burst or a follower that never reports busy cannot re-plan every loop.
        if (in.poseJump && nowMs - lastPoseJumpMs >= CollectConfig.REPLAN_COOLDOWN_MS) {
            return Reason.POSE_JUMP;
        }
        if (in.pathDone && nowMs - lastReplanMs >= CollectConfig.PATH_DONE_MIN_MS) {
            return Reason.PATH_DONE;
        }
        boolean stalled = updateStall(nowMs, in);
        boolean headingOff = updateHeading(nowMs, in);
        if (nowMs - lastReplanMs < CollectConfig.REPLAN_COOLDOWN_MS) {
            return Reason.NONE;
        }
        if (in.targetGone) {
            return Reason.TARGET_GONE;
        }
        if (in.unexpectedPickup) {
            return Reason.UNEXPECTED_PICKUP;
        }
        if (in.missedTarget) {
            return Reason.MISSED_TARGET;
        }
        if (in.offPathIn > CollectConfig.OFF_PATH_IN || headingOff) {
            return Reason.OFF_PATH;
        }
        if (stalled) {
            return Reason.STALLED;
        }
        if (in.betterBall) {
            return Reason.BETTER_BALL;
        }
        return Reason.NONE;
    }

    /**
     * Heading error only counts once it has lasted OFF_PATH_HEADING_MS: Weaver paths are
     * tangent-heading, so right after a re-plan towards a ball beside or behind the robot the
     * error is large but transient while Pedro turns the robot.
     */
    private boolean updateHeading(long nowMs, Inputs in) {
        if (in.headingErrDeg <= CollectConfig.OFF_PATH_DEG) {
            headingErrSinceMs = -1;
            return false;
        }
        if (headingErrSinceMs < 0) {
            headingErrSinceMs = nowMs;
        }
        return nowMs - headingErrSinceMs >= CollectConfig.OFF_PATH_HEADING_MS;
    }

    private boolean updateStall(long nowMs, Inputs in) {
        if (!in.following || in.pose == null) {
            stallSinceMs = -1;
            return false;
        }
        if (stallSinceMs < 0
                || Math.hypot(in.pose.x() - anchorX, in.pose.y() - anchorY) > CollectConfig.STALL_MOVE_IN) {
            anchorX = in.pose.x();
            anchorY = in.pose.y();
            stallSinceMs = nowMs;
            return false;
        }
        return nowMs - stallSinceMs >= CollectConfig.STALL_MS;
    }
}
