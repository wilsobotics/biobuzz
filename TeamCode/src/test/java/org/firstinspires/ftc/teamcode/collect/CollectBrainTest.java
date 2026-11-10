package org.firstinspires.ftc.teamcode.collect;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.perception.BallClass;
import org.firstinspires.ftc.teamcode.perception.BallMap;
import org.junit.Before;
import org.junit.Test;

import java.util.Collections;

public class CollectBrainTest {
    static final class FakeFollower implements PathFollower {
        Pose pose;
        boolean busy;
        boolean instant;   // follow() leaves the path already done
        boolean stopped;
        int follows;

        FakeFollower(Pose pose) {
            this.pose = pose;
        }

        @Override public void follow(Path p) { busy = !instant; stopped = false; follows++; }
        @Override public boolean isBusy() { return busy; }
        @Override public Pose pose() { return pose; }
        @Override public Pose closestPose() { return pose; }
        @Override public void stop() { busy = false; stopped = true; }
        void finish() { busy = false; }
    }

    static final class FakeIntake implements IntakeStatus {
        int count;
        @Override public int ballCount() { return count; }
        @Override public int capacity() { return 4; }
    }

    private BallMap map;
    private FakeFollower follower;
    private FakeIntake intake;

    @Before
    public void setUp() {
        map = new BallMap();
        follower = new FakeFollower(new Pose(20, 20, 0));
        intake = new FakeIntake();
    }

    private CollectBrain brain(CollectRequest req) {
        return new CollectBrain(req, map, follower, intake, new FieldFrame(0),
                Collections.<ObstacleProvider>emptyList());
    }

    private static CollectRequest.Builder quiet() {
        return CollectRequest.builder(Alliance.RED, false).useHints(false).onlySources();
    }

    @Test
    public void startsFollowingAPlanToTheTarget() {
        MapSeeds.confirmed(map, BallClass.POLLEN, 40.0, 20.0);
        CollectBrain b = brain(quiet().build());
        b.start(100);
        assertEquals(1, follower.follows);
        assertEquals(1, b.plannedTargets().size());
        assertFalse(b.isDone());
    }

    @Test
    public void endsFullWhenTheIntakeFills() {
        MapSeeds.confirmed(map, BallClass.POLLEN, 40.0, 20.0);
        CollectBrain b = brain(quiet().build());
        b.start(100);
        intake.count = 4;
        b.update(200);
        assertTrue(b.isDone());
        assertEquals(CollectResult.FULL, b.result());
        assertTrue(follower.stopped);
    }

    @Test
    public void nothingToDoEndsWithNoTargets() {
        CollectBrain b = brain(quiet().build());
        b.start(100);
        b.update(100 + CollectConfig.START_DWELL_MS);   // start dwell over
        assertEquals(CollectResult.NO_TARGETS, b.result());
    }

    @Test
    public void searchesASourceThenGivesUp() {
        CollectBrain b = brain(quiet().onlySources("LOADING_ZONE_RED").build());
        b.start(100);
        long t0 = 100 + CollectConfig.START_DWELL_MS;  // start dwell over
        b.update(t0);
        assertEquals(1, follower.follows);
        assertFalse(b.isDone());
        follower.finish();
        b.update(t0 + 100);
        assertFalse(b.isDone());          // dwelling at the look pose
        b.update(t0 + 600);
        assertEquals(CollectResult.NO_TARGETS, b.result());
    }

    @Test
    public void replansWhenAPlannedTargetDisappears() {
        MapSeeds.confirmed(map, BallClass.POLLEN, 40.0, 20.0, BallClass.POLLEN, 60.0, 30.0);
        CollectBrain b = brain(quiet().build());
        b.start(100);
        map.removePickup(new Pose(40 - 14.3, 20, 0));
        b.update(500);
        assertEquals(2, follower.follows);
        assertEquals(ReplanPolicy.Reason.TARGET_GONE, b.lastReason());
    }

    @Test
    public void skipsATargetAfterTwoMisses() {
        MapSeeds.confirmed(map, BallClass.POLLEN, 40.0, 20.0);
        CollectBrain b = brain(quiet().build());
        b.start(100);
        b.update(200);                    // intake mouth within reach: waiting for a pickup
        b.update(900);                    // grace over, no pickup: miss 1, re-plan
        assertFalse(b.isDone());
        assertTrue(b.skipped().isEmpty());
        b.update(1000);
        b.update(1700);                   // miss 2: skipped, nothing left
        assertEquals(1, b.skipped().size());
        assertEquals(CollectResult.NO_TARGETS, b.result());
    }

    @Test
    public void timesOut() {
        MapSeeds.confirmed(map, BallClass.POLLEN, 40.0, 20.0);
        CollectBrain b = brain(quiet().timeoutSec(1).build());
        b.start(100);
        b.update(1200);
        assertEquals(CollectResult.TIMEOUT, b.result());
    }

    @Test
    public void stopInterrupts() {
        MapSeeds.confirmed(map, BallClass.POLLEN, 40.0, 20.0);
        CollectBrain b = brain(quiet().build());
        b.start(100);
        b.stop();
        assertEquals(CollectResult.INTERRUPTED, b.result());
        assertTrue(follower.stopped);
    }

    @Test
    public void poseJumpForcesAReplan() {
        MapSeeds.confirmed(map, BallClass.POLLEN, 40.0, 20.0);
        CollectBrain b = brain(quiet().build());
        b.start(100);
        follower.pose = new Pose(20, 10, 0);
        b.update(150);
        assertEquals(ReplanPolicy.Reason.POSE_JUMP, b.lastReason());
        assertEquals(2, follower.follows);
    }

    @Test
    public void waitsOutPickupGraceWhenThePathEndsAtTheBall() {
        follower.instant = true;
        MapSeeds.confirmed(map, BallClass.POLLEN, 40.0, 20.0);
        CollectBrain b = brain(quiet().build());
        b.start(100);
        b.update(150);                    // at the ball: grace window opens
        b.update(800);                    // grace over: miss 1 only
        assertTrue(b.skipped().isEmpty());
        for (long t = 850; t <= 3000 && !b.isDone(); t += 50) {
            b.update(t);
        }
        assertEquals(CollectResult.NO_TARGETS, b.result());
        assertEquals(1, b.skipped().size());
        assertTrue("follows=" + follower.follows, follower.follows <= 4);
    }

    @Test
    public void unreachableTargetIsSkippedInsteadOfReplannedForever() {
        MapSeeds.confirmed(map, BallClass.POLLEN, 72.0, 72.0);
        CollectBrain b = brain(quiet().onlySources("LOADING_ZONE_RED").build());
        b.start(100);
        assertEquals("LOOK", b.state());
        assertEquals(1, follower.follows);
        assertTrue(b.skipped().isEmpty());     // held temporarily, not skipped for the run
        b.update(400);
        b.update(800);
        assertEquals(1, follower.follows);     // no re-plan thrash inside the hold window
        assertEquals(1, b.replanCount());
        assertFalse(b.isDone());
    }

    @Test
    public void unreachableBallIsOfferedAgainAfterTheHold() {
        MapSeeds.confirmed(map, BallClass.POLLEN, 72.0, 72.0);
        CollectBrain b = brain(quiet().onlySources("LOADING_ZONE_RED").build());
        b.start(100);
        b.update(800);
        assertEquals(1, b.replanCount());
        b.update(100 + CollectConfig.UNREACHABLE_RETRY_MS + 200);
        assertEquals(2, b.replanCount());      // ball became a candidate again and triggered a re-plan
        assertTrue(b.skipped().isEmpty());
    }

    @Test
    public void poseJumpStillReplansDuringPickupGrace() {
        follower.instant = true;
        MapSeeds.confirmed(map, BallClass.POLLEN, 40.0, 20.0);
        CollectBrain b = brain(quiet().build());
        b.start(100);
        b.update(150);
        follower.pose = new Pose(20, 5, 0);
        b.update(200);
        assertEquals(ReplanPolicy.Reason.POSE_JUMP, b.lastReason());
    }

    @Test
    public void updateBeforeStartDoesNothing() {
        MapSeeds.confirmed(map, BallClass.POLLEN, 40.0, 20.0);
        CollectBrain b = brain(quiet().build());
        b.update(100);
        assertFalse(b.isDone());
        assertEquals(0, follower.follows);
    }

    @Test
    public void slowMoveBetweenUpdatesIsNotAPoseJump() {
        MapSeeds.confirmed(map, BallClass.POLLEN, 40.0, 20.0);
        CollectBrain b = brain(quiet().build());
        b.start(100);
        follower.pose = new Pose(20, 30, 0);     // 10 in over 200 ms = 50 in/s
        b.update(300);
        assertEquals(ReplanPolicy.Reason.NONE, b.lastReason());
        assertEquals(1, follower.follows);
    }

    @Test
    public void fastMoveBetweenUpdatesIsAPoseJump() {
        MapSeeds.confirmed(map, BallClass.POLLEN, 40.0, 20.0);
        CollectBrain b = brain(quiet().build());
        b.start(100);
        follower.pose = new Pose(20, 30, 0);     // 10 in over 20 ms = 500 in/s
        b.update(120);
        assertEquals(ReplanPolicy.Reason.POSE_JUMP, b.lastReason());
        assertEquals(2, follower.follows);
    }

    @Test
    public void twoPoseJumpsCloseTogetherReplanOnce() {
        MapSeeds.confirmed(map, BallClass.POLLEN, 40.0, 20.0);
        CollectBrain b = brain(quiet().build());
        b.start(100);
        follower.pose = new Pose(20, 30, 0);
        b.update(120);
        follower.pose = new Pose(20, 40, 0);
        b.update(170);                           // 50 ms later, another jump
        assertEquals(2, b.replanCount());
    }

    @Test
    public void emptyMapAtStartWaitsForTargetsInsteadOfGivingUp() {
        CollectBrain b = brain(quiet().build());
        b.start(100);
        assertFalse(b.isDone());
        assertTrue(follower.stopped);
        b.update(200);
        assertFalse(b.isDone());
        MapSeeds.confirmed(map, BallClass.POLLEN, 40.0, 20.0);
        b.update(300);
        assertFalse(b.isDone());
        assertEquals("TARGETS", b.state());
        assertEquals(1, follower.follows);
    }

    @Test
    public void heldUnreachableTargetDelaysNoTargetsUntilTheHoldEnds() {
        MapSeeds.confirmed(map, BallClass.POLLEN, 72.0, 72.0);
        CollectBrain b = brain(quiet().build());
        b.start(100);
        assertFalse(b.isDone());
        assertTrue(follower.stopped);
        b.update(500);
        b.update(100 + CollectConfig.UNREACHABLE_RETRY_MS - 50);
        assertFalse(b.isDone());
        assertEquals(1, b.replanCount());
        b.update(100 + CollectConfig.UNREACHABLE_RETRY_MS + 50);
        assertEquals(2, b.replanCount());        // retried once the hold ran out
        assertFalse(b.isDone());
    }
}
