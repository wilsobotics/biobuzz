package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/** Pure math checks for FieldMap - no robot, no hardware. */
public class FieldMapTest {

    private static final double EPS = 1e-6;

    @Test
    public void fieldIsSixTilesAcrossAndCentredOnTheHive() {
        assertEquals(3657.6, FieldMap.FIELD_SIZE_MM, 0.05);
        assertEquals(1828.8, FieldMap.HALF_FIELD_MM, EPS);
        assertEquals(609.6, FieldMap.TILE_MM, EPS);
        assertEquals(1828.8, FieldMap.TILE_MM * 3.0, EPS);   // 6 tiles span the field
    }

    @Test
    public void tileGridMatchesThePublishedLandmarks() {
        // A is against the red wall (+X), row 1 against the audience wall (+Y).
        assertEquals(1524.0, FieldMap.tileColumnCentreMm('A'), EPS);
        assertEquals(-1524.0, FieldMap.tileColumnCentreMm('f'), EPS);
        assertEquals(1524.0, FieldMap.tileRowCentreMm(1), EPS);
        assertEquals(-1524.0, FieldMap.tileRowCentreMm(6), EPS);

        Pose2D f6 = FieldMap.tileCentre("F6");
        assertEquals(-1524.0, f6.getX(DistanceUnit.MM), EPS);
        assertEquals(-1524.0, f6.getY(DistanceUnit.MM), EPS);

        // Red loading zone is on A5 against the red wall, 11 in. deep, so it stops well short of
        // the TILE centre; blue is the same zone half a turn away, on F2.
        assertTrue(FieldMap.RED_LOADING_ZONE.contains(
                FieldMap.HALF_FIELD_MM - 100.0, FieldMap.tileRowCentreMm(5)));
        assertTrue(FieldMap.BLUE_LOADING_ZONE.contains(
                -FieldMap.HALF_FIELD_MM + 100.0, FieldMap.tileRowCentreMm(2)));
        assertFalse(FieldMap.RED_LOADING_ZONE.contains(FieldMap.tileCentre("A5")));
        assertFalse(FieldMap.RED_LOADING_ZONE.contains(
                FieldMap.HALF_FIELD_MM - 100.0, FieldMap.tileRowCentreMm(3)));

        // Gardens sit in the A1 / F6 corners, against the audience and far walls.
        assertEquals(FieldMap.HALF_FIELD_MM,
                FieldMap.RED_GARDEN.getMaxY(), EPS);
        assertEquals(-FieldMap.HALF_FIELD_MM,
                FieldMap.BLUE_GARDEN.getMinY(), EPS);
    }

    @Test
    public void fieldToRobotAtHeadingZeroIsTheIdentity() {
        double[] out = FieldMap.fieldToRobot(1.0, 0.0, 0.0);
        assertEquals(1.0, out[0], EPS);
        assertEquals(0.0, out[1], EPS);

        // Field +Y is left, and the wheel mix wants right, so it must come back negative.
        out = FieldMap.fieldToRobot(0.0, 1.0, 0.0);
        assertEquals(0.0, out[0], EPS);
        assertEquals(-1.0, out[1], EPS);
    }

    @Test
    public void stickForwardFollowsTheRobotHeading() {
        // The start pose faces the field centre (-X), so field +X is behind the robot.
        double[] out = FieldMap.fieldToRobot(1.0, 0.0, FieldMap.START_RED.getHeading(AngleUnit.RADIANS));
        assertEquals(-1.0, out[0], EPS);
        assertEquals(0.0, out[1], EPS);

        // Pushing forward from that pose drives toward field -X.
        double[] field = FieldMap.robotToField(1.0, 0.0, Math.PI);
        assertEquals(-1.0, field[0], EPS);
        assertEquals(0.0, field[1], EPS);

        // Facing field +Y (heading +90), field +X is the robot's right.
        out = FieldMap.fieldToRobot(1.0, 0.0, Math.PI / 2.0);
        assertEquals(0.0, out[0], EPS);
        assertEquals(1.0, out[1], EPS);
    }

    @Test
    public void transformsRoundTrip() {
        double heading = Math.toRadians(37.0);
        double[] robot = FieldMap.fieldToRobot(400.0, -900.0, heading);
        double[] field = FieldMap.robotToField(robot[0], robot[1], heading);
        assertEquals(400.0, field[0], 1e-9);
        assertEquals(-900.0, field[1], 1e-9);
    }

    @Test
    public void rotationPreservesMagnitude() {
        double[] out = FieldMap.fieldToRobot(0.5, -0.5, Math.toRadians(123.0));
        assertEquals(Math.hypot(0.5, -0.5), Math.hypot(out[0], out[1]), EPS);
    }

    @Test
    public void wrapRadiansHandlesTheBoundaries() {
        assertEquals(0.0, FieldMap.wrapRadians(0.0), EPS);
        assertEquals(Math.PI, FieldMap.wrapRadians(Math.PI), EPS);
        assertEquals(-Math.PI / 2.0, FieldMap.wrapRadians(3.0 * Math.PI / 2.0), 1e-9);
        assertEquals(Math.PI / 2.0, FieldMap.wrapRadians(-3.0 * Math.PI / 2.0), 1e-9);
        assertEquals(Math.toRadians(10.0), FieldMap.wrapRadians(Math.toRadians(370.0)), 1e-9);
    }

    @Test
    public void mirroringIsAHalfTurnAndItsOwnInverse() {
        Pose2D blue = FieldMap.startPose(Alliance.BLUE);
        Pose2D red = FieldMap.startPose(Alliance.RED);

        assertEquals(-red.getX(DistanceUnit.MM), blue.getX(DistanceUnit.MM), EPS);
        assertEquals(-red.getY(DistanceUnit.MM), blue.getY(DistanceUnit.MM), EPS);
        assertEquals(0.0, blue.getHeading(AngleUnit.RADIANS), 1e-9);        // faces back at centre

        Pose2D twice = FieldMap.forAlliance(blue, Alliance.BLUE);
        assertEquals(red.getX(DistanceUnit.MM), twice.getX(DistanceUnit.MM), EPS);
        assertEquals(red.getY(DistanceUnit.MM), twice.getY(DistanceUnit.MM), EPS);

        // Mirrored zones land exactly on the published blue zones.
        assertEquals(FieldMap.RED_LOADING_ZONE.centre().getX(DistanceUnit.MM),
                -FieldMap.BLUE_LOADING_ZONE.centre().getX(DistanceUnit.MM), EPS);
        assertEquals(FieldMap.RED_GARDEN.centre().getY(DistanceUnit.MM),
                -FieldMap.BLUE_GARDEN.centre().getY(DistanceUnit.MM), EPS);

        // Each alliance's flower on the audience wall mirrors to its flower on the far wall.
        Pose2D mirrored = FieldMap.landmark("flowerAudienceRed", Alliance.BLUE);
        Pose2D expected = FieldMap.landmark("flowerFarBlue");
        assertEquals(expected.getX(DistanceUnit.MM), mirrored.getX(DistanceUnit.MM), EPS);
        assertEquals(expected.getY(DistanceUnit.MM), mirrored.getY(DistanceUnit.MM), EPS);
        assertEquals(expected.getHeading(AngleUnit.RADIANS),
                mirrored.getHeading(AngleUnit.RADIANS), 1e-9);
    }

    @Test
    public void startPosesAreLegalStarts() {
        for (Alliance alliance : Alliance.values()) {
            Pose2D start = FieldMap.startPose(alliance);
            assertTrue(FieldMap.insideField(start.getX(DistanceUnit.MM), start.getY(DistanceUnit.MM)));

            // Back against the alliance wall.
            double wall = alliance == Alliance.RED ? FieldMap.HALF_FIELD_MM : -FieldMap.HALF_FIELD_MM;
            assertEquals(wall - Math.signum(wall) * FieldMap.START_STANDOFF_FROM_WALL_MM,
                    start.getX(DistanceUnit.MM), EPS);

            // Clear of the alliance's own LOADING ZONE (and of its GARDEN, which is a corner).
            FieldMap.Zone loading = alliance == Alliance.RED
                    ? FieldMap.RED_LOADING_ZONE : FieldMap.BLUE_LOADING_ZONE;
            FieldMap.Zone garden = alliance == Alliance.RED
                    ? FieldMap.RED_GARDEN : FieldMap.BLUE_GARDEN;
            assertFalse(loading.contains(start));
            assertFalse(garden.contains(start));

            // On the alliance's side of the halfway line (G303 columns A-C or D-F).
            assertTrue(alliance == Alliance.RED
                    ? start.getX(DistanceUnit.MM) > 0 : start.getX(DistanceUnit.MM) < 0);
        }
    }

    @Test(expected = IllegalArgumentException.class)
    public void unknownLandmarkThrowsInsteadOfReturningZero() {
        FieldMap.landmark("noSuchThing");
    }
}
