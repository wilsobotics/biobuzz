package org.firstinspires.ftc.teamcode.collect;

import static org.junit.Assert.assertEquals;

import com.pedropathing.math.Pose;

import org.junit.Before;
import org.junit.Test;

public class ReplanPolicyTest {
    private ReplanPolicy policy;
    private ReplanPolicy.Inputs in;

    @Before
    public void setUp() {
        policy = new ReplanPolicy();
        policy.reset(0);
        in = new ReplanPolicy.Inputs();
        in.following = true;
        in.pose = new Pose(0, 0, 0);
    }

    @Test
    public void cooldownHoldsBackEvents() {
        in.targetGone = true;
        assertEquals(ReplanPolicy.Reason.NONE, policy.check(100, in));
        assertEquals(ReplanPolicy.Reason.TARGET_GONE, policy.check(300, in));
        assertEquals(ReplanPolicy.Reason.NONE, policy.check(400, in));
    }

    @Test
    public void pathDoneAndPoseJumpIgnoreTheCooldown() {
        in.pathDone = true;
        assertEquals(ReplanPolicy.Reason.PATH_DONE, policy.check(CollectConfig.PATH_DONE_MIN_MS, in));
        in.pathDone = false;
        in.poseJump = true;
        assertEquals(ReplanPolicy.Reason.POSE_JUMP, policy.check(CollectConfig.PATH_DONE_MIN_MS + 10, in));
    }

    @Test
    public void offPathByDistanceOrHeading() {
        in.offPathIn = 7;
        assertEquals(ReplanPolicy.Reason.OFF_PATH, policy.check(400, in));
        in.offPathIn = 0;
        in.headingErrDeg = 25;
        policy.check(800, in);            // heading error must persist (OFF_PATH_HEADING_MS)
        assertEquals(ReplanPolicy.Reason.OFF_PATH, policy.check(800 + CollectConfig.OFF_PATH_HEADING_MS, in));
    }

    @Test
    public void stallFiresOnlyWhenNotMoving() {
        assertEquals(ReplanPolicy.Reason.NONE, policy.check(400, in));
        assertEquals(ReplanPolicy.Reason.NONE, policy.check(700, in));
        assertEquals(ReplanPolicy.Reason.STALLED, policy.check(900, in));
        in.pose = new Pose(0, 0, 0);
        assertEquals(ReplanPolicy.Reason.NONE, policy.check(1300, in));
        in.pose = new Pose(5, 0, 0);
        assertEquals(ReplanPolicy.Reason.NONE, policy.check(1500, in));
        in.pose = new Pose(10, 0, 0);
        assertEquals(ReplanPolicy.Reason.NONE, policy.check(1900, in));
    }

    @Test
    public void eventPriorityOrder() {
        in.betterBall = true;
        in.missedTarget = true;
        assertEquals(ReplanPolicy.Reason.MISSED_TARGET, policy.check(400, in));
    }

    @Test
    public void poseJumpsHaveAMinimumInterval() {
        in.poseJump = true;
        assertEquals(ReplanPolicy.Reason.POSE_JUMP, policy.check(400, in));
        assertEquals(ReplanPolicy.Reason.NONE, policy.check(450, in));
        assertEquals(ReplanPolicy.Reason.POSE_JUMP, policy.check(400 + CollectConfig.REPLAN_COOLDOWN_MS, in));
    }

    @Test
    public void headingErrorMustPersistBeforeOffPath() {
        in.headingErrDeg = 30;
        in.pose = new Pose(0, 0, 0);
        assertEquals(ReplanPolicy.Reason.NONE, policy.check(400, in));
        in.pose = new Pose(5, 0, 0);
        assertEquals(ReplanPolicy.Reason.NONE, policy.check(700, in));      // 300 ms
        in.pose = new Pose(10, 0, 0);
        assertEquals(ReplanPolicy.Reason.OFF_PATH, policy.check(1100, in)); // 700 ms
    }

    @Test
    public void headingPersistenceRestartsWhenTheErrorClears() {
        in.headingErrDeg = 30;
        policy.check(400, in);
        in.headingErrDeg = 0;
        in.pose = new Pose(5, 0, 0);
        policy.check(700, in);
        in.headingErrDeg = 30;
        in.pose = new Pose(10, 0, 0);
        assertEquals(ReplanPolicy.Reason.NONE, policy.check(1100, in));
    }

    @Test
    public void pathDoneHasAMinimumIntervalSinceTheLastReplan() {
        in.pathDone = true;
        assertEquals(ReplanPolicy.Reason.PATH_DONE, policy.check(400, in));
        assertEquals(ReplanPolicy.Reason.NONE, policy.check(450, in));
        assertEquals(ReplanPolicy.Reason.PATH_DONE, policy.check(400 + CollectConfig.PATH_DONE_MIN_MS, in));
    }
}
