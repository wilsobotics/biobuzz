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
}
