package org.firstinspires.ftc.teamcode.perception;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.EnumSet;
import java.util.List;

public class BallMapTest {
    private static final CameraMount M = new CameraMount(0, 0, 12.4, 25, 0, 54.5, 42);

    static Sighting s(double x, double y, double sigma, double quality) {
        return new Sighting(BallClass.POLLEN, x, y, 40, sigma, quality, false);
    }

    static List<Sighting> list(Sighting... ss) {
        return new ArrayList<Sighting>(Arrays.asList(ss));
    }

    @Test
    public void twoSightingsInOneFrameAreNeverMerged() {
        BallMap map = new BallMap();
        map.update(0, null, null, list(s(40, 0, 1.0, 0.9), s(40, 1.0, 1.0, 0.9)));
        assertEquals(2, map.size());
    }

    @Test
    public void touchingPollenStaySeparateAcrossFrames() {
        BallMap map = new BallMap();
        map.update(0, null, null, list(s(40, 0, 1.0, 0.9), s(40, 2.8, 1.0, 0.9)));
        map.update(50, null, null, list(s(40, 0.3, 1.0, 0.9), s(40, 3.1, 1.0, 0.9)));
        map.update(100, null, null, list(s(40, -0.3, 1.0, 0.9), s(40, 2.5, 1.0, 0.9)));
        assertEquals(2, map.size());
        assertTrue(map.all().get(0).isConfirmed());
        assertTrue(map.all().get(1).isConfirmed());
    }

    @Test
    public void newNeighbourOfAConfirmedBallBecomesItsOwnBall() {
        BallMap map = new BallMap();
        map.update(0, null, null, list(s(40, 0, 1.0, 0.9)));
        map.update(50, null, null, list(s(40, 0, 1.0, 0.9)));
        map.update(100, null, null, list(s(40, 0.2, 1.0, 0.9), s(40, 2.6, 1.0, 0.9)));
        assertEquals(2, map.size());
    }

    @Test
    public void farHintSplitsWhenSeenUpClose() {
        BallMap map = new BallMap();
        map.update(0, null, null, list(s(100, 0, 6.0, 0.25)));
        map.update(50, null, null, list(s(100, 0, 6.0, 0.25)));
        assertEquals(1, map.hints().size());
        map.update(100, null, null, list(s(100, -1.4, 1.0, 0.9), s(100, 1.4, 1.0, 0.9)));
        assertEquals(2, map.size());
    }

    @Test
    public void aBallNeedsTwoFramesToBeConfirmed() {
        BallMap map = new BallMap();
        map.update(0, null, null, list(s(40, 0, 1.0, 0.9)));
        assertFalse(map.all().get(0).isConfirmed());
        assertTrue(map.targets().isEmpty());
        map.update(50, null, null, list(s(40, 0.1, 1.0, 0.9)));
        assertTrue(map.all().get(0).isConfirmed());
        assertEquals(1, map.targets().size());
    }

    @Test
    public void unconfirmedCandidatesExpire() {
        BallMap map = new BallMap();
        map.update(0, null, null, list(s(40, 0, 1.0, 0.9)));
        map.prune(600);
        assertEquals(0, map.size());
    }

    @Test
    public void confirmedBallsAreForgottenAfterFourSeconds() {
        BallMap map = new BallMap();
        map.update(0, null, null, list(s(40, 0, 1.0, 0.9)));
        map.update(50, null, null, list(s(40, 0, 1.0, 0.9)));
        map.prune(4000);
        assertEquals(1, map.size());
        map.prune(4100);
        assertEquals(0, map.size());
    }

    @Test
    public void ballInViewButNotSeenIsRemovedAfterTwoFrames() {
        BallMap map = new BallMap();
        Pose robot = new Pose(0, 0, 0);
        map.update(0, robot, M, list(s(40, 0, 1.0, 0.9)));
        map.update(50, robot, M, list(s(40, 0, 1.0, 0.9)));
        map.update(100, robot, M, Collections.<Sighting>emptyList());
        assertEquals(1, map.size());
        map.update(150, robot, M, Collections.<Sighting>emptyList());
        assertEquals(0, map.size());
    }

    @Test
    public void ballOutOfViewIsKept() {
        BallMap map = new BallMap();
        map.update(0, new Pose(0, 0, 0), M, list(s(40, 0, 1.0, 0.9)));
        map.update(50, new Pose(0, 0, 0), M, list(s(40, 0, 1.0, 0.9)));
        Pose facingAway = new Pose(0, 0, Math.PI);
        map.update(100, facingAway, M, Collections.<Sighting>emptyList());
        map.update(150, facingAway, M, Collections.<Sighting>emptyList());
        assertEquals(1, map.size());
    }

    @Test
    public void pickupRemovesExactlyOneBall() {
        BallMap map = new BallMap();
        map.update(0, null, null, list(s(15, 0, 1.0, 0.9), s(15, 2.8, 1.0, 0.9)));
        map.update(50, null, null, list(s(15, 0, 1.0, 0.9), s(15, 2.8, 1.0, 0.9)));
        MapBall taken = map.removePickup(new Pose(0, 0, 0));
        assertNotNull(taken);
        assertEquals(15, taken.x(), 0.2);
        assertEquals(0, taken.y(), 0.2);
        assertEquals(1, map.size());
        assertEquals(2.8, map.all().get(0).y(), 0.2);
    }

    @Test
    public void maybeTwoBallSurvivesTheFirstPickup() {
        BallMap map = new BallMap();
        Sighting two = new Sighting(BallClass.POLLEN, 15, 0, 15, 1.0, 0.9, true);
        map.update(0, null, null, list(two));
        map.update(50, null, null, list(two));
        assertNotNull(map.removePickup(new Pose(0, 0, 0)));
        assertEquals(1, map.size());
        assertFalse(map.all().get(0).maybeTwo());
        assertNotNull(map.removePickup(new Pose(0, 0, 0)));
        assertEquals(0, map.size());
    }

    @Test
    public void differentClassesNeverMatch() {
        BallMap map = new BallMap();
        map.update(0, null, null, list(s(40, 0, 1.0, 0.9)));
        map.update(50, null, null, list(new Sighting(BallClass.RED_NECTAR, 40, 0, 40, 1.0, 0.9, false)));
        assertEquals(2, map.size());
    }

    @Test
    public void preciseBallAndImpreciseNeighbourStaySeparate() {
        BallMap map = new BallMap();
        map.update(0, null, null, list(s(40, 0, 1.0, 0.9)));
        map.update(50, null, null, list(s(40, 0, 1.0, 0.9)));
        map.update(100, null, null, list(s(40, 8, 6.0, 0.25)));
        assertEquals(2, map.size());
    }

    @Test
    public void impreciseHintAndPreciseNeighbourStaySeparate() {
        BallMap map = new BallMap();
        map.update(0, null, null, list(s(100, 0, 6.0, 0.25)));
        map.update(50, null, null, list(s(100, 0, 6.0, 0.25)));
        map.update(100, null, null, list(s(100, 8, 1.0, 0.9)));
        assertEquals(2, map.size());
    }

    @Test
    public void missesMustBeConsecutiveInView() {
        BallMap map = new BallMap();
        Pose robot = new Pose(0, 0, 0);
        map.update(0, robot, M, list(s(40, 0, 1.0, 0.9)));
        map.update(50, robot, M, list(s(40, 0, 1.0, 0.9)));
        map.update(100, robot, M, Collections.<Sighting>emptyList());
        map.update(150, new Pose(0, 0, Math.PI), M, Collections.<Sighting>emptyList());
        map.update(200, robot, M, Collections.<Sighting>emptyList());
        assertEquals(1, map.size());
    }

    @Test
    public void staleCandidateCannotBeRevivedByALateFrame() {
        BallMap map = new BallMap();
        map.update(0, null, null, list(s(40, 0, 1.0, 0.9)));
        map.update(600, null, null, list(s(40, 0, 1.0, 0.9)));
        assertEquals(1, map.size());
        assertEquals(1, map.all().get(0).seenCount());
        assertFalse(map.all().get(0).isConfirmed());
    }

    @Test
    public void zeroSigmaSightingDoesNotProduceNaN() {
        BallMap map = new BallMap();
        map.update(0, null, null, list(s(40, 0, 0.0, 0.9)));
        map.update(50, null, null, list(s(40, 0, 0.0, 0.9)));
        MapBall b = map.all().get(0);
        assertEquals(40, b.x(), 1e-9);
        assertTrue(b.sigma() >= PerceptionConfig.MIN_SIGMA_IN);
    }

    @Test
    public void missesOnlyCountForClassesThePipelineCanSee() {
        BallMap map = new BallMap();
        Pose robot = new Pose(0, 0, 0);
        Sighting red = new Sighting(BallClass.RED_NECTAR, 40, 0, 40, 1.0, 0.9, false);
        map.update(0, robot, M, list(red));
        map.update(50, robot, M, list(red));
        assertEquals(1, map.size());

        map.update(100, robot, M, Collections.<Sighting>emptyList(), EnumSet.of(BallClass.POLLEN));
        map.update(150, robot, M, Collections.<Sighting>emptyList(), EnumSet.of(BallClass.POLLEN));
        assertEquals(1, map.size());

        map.update(200, robot, M, Collections.<Sighting>emptyList(), EnumSet.of(BallClass.RED_NECTAR));
        map.update(250, robot, M, Collections.<Sighting>emptyList(), EnumSet.of(BallClass.RED_NECTAR));
        assertEquals(0, map.size());
    }
    @Test
    public void ballHiddenBehindAMatchedBallIsNotDeleted() {
        BallMap map = new BallMap();
        Pose robot = new Pose(0, 0, 0);
        map.update(0, robot, M, list(s(35, 0, 1.0, 0.9), s(45, 0, 1.0, 0.9)));
        map.update(50, robot, M, list(s(35, 0, 1.0, 0.9), s(45, 0, 1.0, 0.9)));
        assertEquals(2, map.targets().size());
        map.update(100, robot, M, list(s(35, 0, 1.0, 0.9)));
        map.update(150, robot, M, list(s(35, 0, 1.0, 0.9)));
        map.update(200, robot, M, list(s(35, 0, 1.0, 0.9)));
        assertEquals(2, map.size());
    }

    @Test
    public void ballBesideAMatchedBallIsStillRemovedWhenMissing() {
        BallMap map = new BallMap();
        Pose robot = new Pose(0, 0, 0);
        map.update(0, robot, M, list(s(35, 0, 1.0, 0.9), s(45, 12, 1.0, 0.9)));
        map.update(50, robot, M, list(s(35, 0, 1.0, 0.9), s(45, 12, 1.0, 0.9)));
        map.update(100, robot, M, list(s(35, 0, 1.0, 0.9)));
        map.update(150, robot, M, list(s(35, 0, 1.0, 0.9)));
        assertEquals(1, map.size());
    }
    @Test
    public void pollenHiddenBehindARecentlySeenNectarSurvivesPollenOnlyFrames() {
        BallMap map = new BallMap();
        Pose robot = new Pose(0, 0, 0);
        Sighting red = new Sighting(BallClass.RED_NECTAR, 35, 0, 35, 1.0, 0.9, false);
        Sighting pollen = s(45, 0, 1.0, 0.9);
        map.update(0, robot, M, list(red, pollen));
        map.update(50, robot, M, list(red, pollen));
        assertEquals(2, map.targets().size());
        // colour cycling: POLLEN-only frames cannot report the nearer red ball that hides the pollen
        for (long t = 100; t <= 300; t += 50) {
            map.update(t, robot, M, Collections.<Sighting>emptyList(), EnumSet.of(BallClass.POLLEN));
        }
        assertEquals(2, map.size());
    }

    @Test
    public void equalResolutionSightingCanClearMaybeTwo() {
        BallMap map = new BallMap();
        map.update(0, null, null, list(new Sighting(BallClass.POLLEN, 40, 0, 40, 1.0, 0.9, true)));
        map.update(50, null, null, list(new Sighting(BallClass.POLLEN, 40, 0, 40, 1.0, 0.9, true)));
        assertTrue(map.all().get(0).isConfirmed());
        assertTrue(map.all().get(0).maybeTwo());
        map.update(100, null, null, list(new Sighting(BallClass.POLLEN, 40, 0, 40, 1.0, 0.9, false)));
        assertEquals(1, map.size());
        assertFalse(map.all().get(0).maybeTwo());
    }
}
