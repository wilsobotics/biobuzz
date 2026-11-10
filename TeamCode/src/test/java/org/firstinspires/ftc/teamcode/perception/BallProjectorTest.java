package org.firstinspires.ftc.teamcode.perception;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;

import org.junit.Test;

public class BallProjectorTest {
    private static final CameraMount PITCH30 = new CameraMount(0, 0, 12.4, 30, 0, 54.5, 42);
    private static final CameraMount LEVEL = new CameraMount(0, 0, 12.4, 0, 0, 54.5, 42);

    @Test
    public void opticalAxisHitsTheFloorAtHeightOverTanPitch() {
        double[] p = BallProjector.project(new Pose(0, 0, 0), PITCH30, 0, 0, 1.4);
        assertEquals(11.0 / Math.tan(Math.toRadians(30)), p[0], 1e-6);
        assertEquals(0, p[1], 1e-6);
        assertEquals(p[0], p[2], 1e-6);
    }

    @Test
    public void tySetsTheRange() {
        double ty = 30 - Math.toDegrees(Math.atan2(11, 40));
        double[] p = BallProjector.project(new Pose(0, 0, 0), PITCH30, 0, ty, 1.4);
        assertEquals(40, p[0], 1e-6);
        assertEquals(0, p[1], 1e-6);
    }

    @Test
    public void positiveTxIsToTheRight() {
        double tx = Math.toDegrees(Math.atan(10.0 / 40.0));
        double ty = Math.toDegrees(Math.atan(-11.0 / 40.0));
        double[] p = BallProjector.project(new Pose(0, 0, 0), LEVEL, tx, ty, 1.4);
        assertEquals(40, p[0], 1e-6);
        assertEquals(-10, p[1], 1e-6);
    }

    @Test
    public void cameraOffsetAndRobotHeadingAreApplied() {
        CameraMount m = new CameraMount(5, 0, 12.4, 30, 0, 54.5, 42);
        double[] p = BallProjector.project(new Pose(10, 20, Math.PI / 2), m, 0, 0, 1.4);
        assertEquals(10, p[0], 1e-6);
        assertEquals(25 + 11.0 / Math.tan(Math.toRadians(30)), p[1], 1e-6);
    }

    @Test
    public void rayAboveTheHorizonIsNull() {
        assertNull(BallProjector.project(new Pose(0, 0, 0), LEVEL, 0, 5, 1.4));
    }

    @Test
    public void errorPerDegreeMatchesTheFormulaAndGrowsWithRange() {
        assertEquals(1721.0 / 11.0 * Math.PI / 180.0, BallProjector.errorPerDegree(40, 12.4, 1.4), 1e-9);
        assertTrue(BallProjector.errorPerDegree(80, 12.4, 1.4) > 3 * BallProjector.errorPerDegree(40, 12.4, 1.4));
    }

    @Test
    public void classIdsAreLoadBearing() {
        assertEquals(BallClass.POLLEN, BallClass.fromClassId(0));
        assertEquals(BallClass.RED_NECTAR, BallClass.fromClassId(1));
        assertEquals(BallClass.BLUE_NECTAR, BallClass.fromClassId(2));
        assertNull(BallClass.fromClassId(3));
        assertEquals(1.4, BallClass.POLLEN.centerHeightIn(), 1e-9);
    }

    @Test
    public void wrapKeepsAnglesInRange() {
        assertEquals(Math.PI, Angles.wrap(Math.PI), 1e-12);
        assertEquals(-Math.PI / 2, Angles.wrap(3 * Math.PI / 2), 1e-12);
    }
    @Test(timeout = 1000)
    public void wrapDoesNotHangOnNonFiniteOrHugeAngles() {
        Angles.wrap(Double.POSITIVE_INFINITY);
        Angles.wrap(Double.NaN);
        double w = Angles.wrap(1e9);
        assertTrue(w > -Math.PI && w <= Math.PI);
        assertEquals(Math.PI, Angles.wrap(-Math.PI), 1e-12);
        assertEquals(1e9 - 2 * Math.PI * Math.rint(1e9 / (2 * Math.PI)), w, 1e-6);
    }
}
