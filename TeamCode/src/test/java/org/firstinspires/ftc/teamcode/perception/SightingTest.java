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
