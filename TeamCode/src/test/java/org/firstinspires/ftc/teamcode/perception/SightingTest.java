package org.firstinspires.ftc.teamcode.perception;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;

import org.junit.Test;

import java.util.Arrays;
import java.util.List;

public class SightingTest {
    private static final CameraMount M = new CameraMount(0, 0, 12.4, 25, 0, 54.5, 42);
    private static final Pose ORIGIN = new Pose(0, 0, 0);

    private static double tyForRange(double range) {
        return 25 - Math.toDegrees(Math.atan2(11, range));
    }

    @Test
    public void lowConfidenceAndNonBallsAreDropped() {
        assertNull(Sighting.from(new Detection(0, 0.1, 0, tyForRange(40), 1), ORIGIN, M));
        assertNull(Sighting.from(new Detection(7, 0.9, 0, tyForRange(40), 1), ORIGIN, M));
    }

    @Test
    public void closeCentredSightingKeepsItsConfidenceAsQuality() {
        Sighting s = Sighting.from(new Detection(0, 0.9, 0, tyForRange(40), 1), ORIGIN, M);
        assertEquals(40, s.x, 1e-6);
        assertEquals(0.9, s.quality, 1e-9);
        assertEquals(BallProjector.errorPerDegree(40, 12.4, 1.4) * 0.5, s.sigma, 1e-9);
        assertFalse(s.maybeTwo);
    }

    @Test
    public void farAndEdgeSightingsLoseQuality() {
        assertEquals(0.9 * 0.3, Sighting.quality(0.9, 200, 0, 0, M), 1e-9);
        assertEquals(0.9 * 0.65, Sighting.quality(0.9, 84, 0, 0, M), 1e-9);
        assertEquals(0.9 * 0.6, Sighting.quality(0.9, 30, 26, 0, M), 1e-9);
    }

    @Test
    public void wideBoxesAreFlaggedAsMaybeTwo() {
        Sighting s = Sighting.from(new Detection(0, 0.9, 0, tyForRange(40), 2.0), ORIGIN, M);
        assertTrue(s.maybeTwo);
    }

    /** Limelight ta (% of image area) for a box of sizeDeg x sizeDeg*aspect degrees. */
    private static double areaPct(double heightDeg, double aspect) {
        return heightDeg * heightDeg * aspect / (M.hfovDeg * M.vfovDeg) * 100.0;
    }

    @Test
    public void floorBallWhoseBoxMatchesItsRangeIsKept() {
        double expect = Sighting.expectedSizeDeg(40, M, BallClass.POLLEN);
        assertEquals(Math.toDegrees(2 * Math.atan(1.4 / Math.hypot(40, 11))), expect, 1e-9);
        Sighting s = Sighting.from(new Detection(0, 0.9, 0, tyForRange(40), 1, areaPct(expect * 1.2, 1)), ORIGIN, M);
        assertEquals(40, s.x, 1e-6);
    }

    @Test
    public void ballLiftedOffTheFloorIsDropped() {
        // Pollen held 6 in up (centre 7.4 in) at 30 in, e.g. in another robot's intake. Projected
        // onto the floor it lands at about 66 in, where a floor ball would look far smaller.
        double ty = 25 - Math.toDegrees(Math.atan2(12.4 - 7.4, 30));
        double seen = Math.toDegrees(2 * Math.atan(1.4 / Math.hypot(30, 5)));
        assertNull(Sighting.from(new Detection(0, 0.9, 0, ty, 1, areaPct(seen, 1)), ORIGIN, M));
    }

    @Test
    public void sizeCheckIgnoresUnknownSizeMergedPairsAndPartlyHiddenBalls() {
        double ty = tyForRange(40);
        double expect = Sighting.expectedSizeDeg(40, M, BallClass.POLLEN);
        assertTrue(Sighting.from(new Detection(0, 0.9, 0, ty, 1), ORIGIN, M) != null);              // no ta
        Sighting pair = Sighting.from(new Detection(0, 0.9, 0, ty, 2.0, areaPct(expect, 2.0)), ORIGIN, M);
        assertTrue(pair != null && pair.maybeTwo);                                                  // two side by side
        assertTrue(Sighting.from(new Detection(0, 0.9, 0, ty, 1, areaPct(expect * 0.5, 1)), ORIGIN, M) != null);
    }

    @Test
    public void sizeCheckCanBeSwitchedOff() {
        double ty = 25 - Math.toDegrees(Math.atan2(5, 30));
        double seen = Math.toDegrees(2 * Math.atan(1.4 / Math.hypot(30, 5)));
        double saved = PerceptionConfig.MAX_SIZE_RATIO;
        PerceptionConfig.MAX_SIZE_RATIO = 0;
        try {
            assertTrue(Sighting.from(new Detection(0, 0.9, 0, ty, 1, areaPct(seen, 1)), ORIGIN, M) != null);
        } finally {
            PerceptionConfig.MAX_SIZE_RATIO = saved;
        }
    }

    @Test
    public void aspectComesFromBoxCorners() {
        List<List<Double>> corners = Arrays.asList(
                Arrays.asList(10.0, 10.0), Arrays.asList(30.0, 10.0),
                Arrays.asList(30.0, 20.0), Arrays.asList(10.0, 20.0));
        assertEquals(2.0, Detection.aspectFromCorners(corners), 1e-9);
        assertEquals(1.0, Detection.aspectFromCorners(null), 1e-9);
    }

    @Test
    public void cameraViewMatchesTheFieldOfView() {
        assertTrue(CameraView.canSee(ORIGIN, M, 40, 0, 1.4, 60));
        assertFalse(CameraView.canSee(new Pose(0, 0, Math.PI), M, 40, 0, 1.4, 60));
        assertFalse(CameraView.canSee(ORIGIN, M, 10, 0, 1.4, 60));   // below the bottom edge
        assertFalse(CameraView.canSee(ORIGIN, M, 40, 30, 1.4, 60));  // left of the frame
        assertFalse(CameraView.canSee(ORIGIN, M, 70, 0, 1.4, 60));   // beyond maxRange
    }
    @Test
    public void cameraViewIsTheExactInverseOfTheProjector() {
        double big = 1e6;
        double hLim = M.hfovDeg / 2 - PerceptionConfig.EDGE_MARGIN_DEG;  // 23.25
        double vLim = M.vfovDeg / 2 - PerceptionConfig.EDGE_MARGIN_DEG;  // 17
        double[] txs = {-22, -10, 0, 10, 22, -hLim + 0.25, hLim - 0.25};
        double[] tys = {-16, -8, 0, 8, 16, -vLim + 0.25, vLim - 0.25};
        for (double tx : txs) {
            for (double ty : tys) {
                double[] p = BallProjector.project(ORIGIN, M, tx, ty, 1.4);
                assertTrue("projects tx=" + tx + " ty=" + ty, p != null);
                assertTrue("inside tx=" + tx + " ty=" + ty, CameraView.canSee(ORIGIN, M, p[0], p[1], 1.4, big));
            }
        }
        double[][] outside = {
                {26, 0}, {-26, 0}, {hLim + 0.25, 0}, {-hLim - 0.25, 16}, {hLim + 0.25, -16},
                {0, vLim + 0.25}, {0, -vLim - 0.25}, {22, vLim + 0.25}, {-22, -vLim - 0.25}, {22, -18}, {-26, 16}};
        for (double[] o : outside) {
            double[] p = BallProjector.project(ORIGIN, M, o[0], o[1], 1.4);
            assertTrue("projects tx=" + o[0] + " ty=" + o[1], p != null);
            assertFalse("outside tx=" + o[0] + " ty=" + o[1], CameraView.canSee(ORIGIN, M, p[0], p[1], 1.4, big));
        }
    }
}
