package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 * The BIOBUZZ field frame, its published geometry, and the conversions between the field frame
 * and the robot frame. This is the single place that owns "where things are on the field".
 *
 * <h2>Frame</h2>
 * <pre>
 *   Origin     centre of the FIELD, which is also the centre of the HIVE Structure
 *   +X         toward the RED alliance wall (the wall the red ALLIANCE AREA is behind)
 *   +Y         90 deg counter-clockwise from +X, i.e. toward the AUDIENCE wall
 *   +heading   counter-clockwise, 0 = +X
 *   Units      millimetres and radians throughout
 * </pre>
 *
 * The field is 180 deg rotationally symmetric (BIOBUZZ has the same two alliances on opposite
 * walls, and every zone pairs up: red LOADING ZONE on TILE A5 with the blue one on F2, red GARDEN
 * on A1 with the blue one on F6). That is why {@link #forAlliance} mirrors with a half turn rather
 * than a reflection, and why every constant here is written in the RED frame once.
 *
 * <h2>Tiles</h2>
 * TILE columns run A..F from the RED wall to the BLUE wall and rows 1..6 from the AUDIENCE wall to
 * the far ("scoring") wall, so A1 is the red/audience corner and F6 the blue/far corner. Both of
 * those corner TILEs hold a GARDEN, which is how the grid orientation above is pinned down.
 *
 * <h2>Sources</h2>
 * 2026-2027 BIOBUZZ Competition Manual, section 9 ARENA (V TU10) for the FIELD, zones and HIVE /
 * FLOWER dimensions; 2026-2027 Field Acceptance Checklist (rev 26-27.3) for the controlled
 * dimensions; Event Field Setup Guide V1.0 for the TILEs each LOADING ZONE and GARDEN sits on.
 * The manual quotes some of these in inches and some in centimetres; the inch figures are
 * converted here and the centimetre figure is kept in the comment. FIELD and TILE sizes carry a
 * published tolerance of +/- 1 in. (+/- 2.54 cm) - design around that, do not trust these to the
 * millimetre.
 *
 * <h2>What to edit</h2>
 * The geometry constants (FIELD_SIZE_MM, TILE_MM, the zone sizes) come from the manual and only
 * change with a new game. The team choices are START_RED / START_STANDOFF_FROM_WALL_MM (where you
 * put the robot down) and FLOWER_STATION_X_MM / FLOWER_RING_FROM_WALL_MM (where the FLOWERs
 * actually are on your field). Everything else here should be asked for, not guessed at.
 *
 * <h2>What is NOT published</h2>
 * The manual fixes the four FLOWERs to the AUDIENCE and far walls, two per wall (one per alliance
 * half), and gives their height and opening - but not their station along the wall. The values
 * below are the centre of each alliance half and must be confirmed against Figure 2 of the Field
 * Acceptance Checklist (marked TODO(robot)). The two HIVE pivots likewise sit somewhere along the
 * frame's 1256.5 mm width, but their spacing is not quoted anywhere, so only the structure centre
 * and the CELL geometry are modelled.
 */
public final class FieldMap {

    private FieldMap() {
    }

    public static final double IN_MM = 25.4;

    // ------------------------------------------------------------------
    // FIELD (manual 9.2)
    // ------------------------------------------------------------------

    /** Inside the perimeter walls. Manual: "approximately 144 in.", i.e. ~365.75 cm. */
    public static final double FIELD_SIZE_MM = 144.0 * IN_MM;              // 3657.6
    public static final double HALF_FIELD_MM = FIELD_SIZE_MM / 2.0;        // 1828.8

    /** 36 TILEs in a 6 by 6 grid. Manual: "approximately 24 in." by 24 in., i.e. ~60.95 cm. */
    public static final double TILE_MM = 24.0 * IN_MM;                     // 609.6
    public static final int TILES_PER_SIDE = 6;

    // ------------------------------------------------------------------
    // Areas, zones and markings (manual 9.3, setup guide 8.3-8.5)
    // ------------------------------------------------------------------

    /** LOADING ZONE: 23 in. along the wall by 11 in. from the inside face of the wall. */
    public static final double LOADING_ZONE_WIDTH_MM = 23.0 * IN_MM;       // 584.2
    public static final double LOADING_ZONE_DEPTH_MM = 11.0 * IN_MM;       // 279.4

    /** GARDEN: 23 in. of a TILE's width by 2 in. of tape against the wall. */
    public static final double GARDEN_WIDTH_MM = 23.0 * IN_MM;             // 584.2
    public static final double GARDEN_DEPTH_MM = 2.0 * IN_MM;              // 50.8

    /**
     * ALLIANCE AREA: manual says "approximately 97 in. wide by 54 in. deep", outside the FIELD.
     * The setup guide pins the width to TILE seam 1 and seam 5, which are 5 TILEs (120 in.) apart;
     * the 97 in. figure is the taped version of that. Kept as published.
     */
    public static final double ALLIANCE_AREA_WIDTH_MM = 97.0 * IN_MM;      // 2463.8
    public static final double ALLIANCE_AREA_DEPTH_MM = 54.0 * IN_MM;      // 1371.6

    // ------------------------------------------------------------------
    // HIVE Structure (manual 9.6)
    // ------------------------------------------------------------------

    /** Centre of the FIELD, which is where the frame is mounted. */
    public static final Pose2D HIVE_CENTRE = pose(0, 0, 0);

    /** Frame footprint: 49.46 in. across the two pivots, 38.95 in. along the CELL swing. */
    public static final double HIVE_FRAME_WIDTH_MM = 1256.5;
    public static final double HIVE_FRAME_DEPTH_MM = 989.5;

    /** Pivot axis above the TILEs: 43.95 in. */
    public static final double HIVE_PIVOT_HEIGHT_MM = 1116.5;

    /** The two CELLs of one HIVE: 18.8 in. apart, so each sits 239 mm either side of the pivot. */
    public static final double HIVE_CELL_SEPARATION_MM = 478.0;

    /** CELL opening: approximately 20 in. wide by 14 in. tall by 12 in. deep. */
    public static final double HIVE_CELL_OPENING_WIDTH_MM = 508.0;
    public static final double HIVE_CELL_OPENING_HEIGHT_MM = 356.0;
    public static final double HIVE_CELL_OPENING_DEPTH_MM = 305.0;

    /** Checklist 1.13: bottom of the upright CELL opening is 53.5 in. above the TILEs. */
    public static final double HIVE_CELL_BOTTOM_HEIGHT_MM = 1359.0;

    /** Mid-height of the upright CELL opening - the aim point for a launcher. */
    public static final double HIVE_CELL_AIM_HEIGHT_MM =
            HIVE_CELL_BOTTOM_HEIGHT_MM + HIVE_CELL_OPENING_HEIGHT_MM / 2.0;

    /**
     * Which side of its pivot each alliance's start-of-match upward CELL is on: the red CELL is up
     * on the AUDIENCE side and the blue CELL is up on the far side (setup guide 11.1 / reset guide
     * E.i). The downward CELL is the one "pointing at" a FLOWER on those walls.
     */
    public static final double HIVE_RED_UP_CELL_SIDE = +1.0;   // +Y, audience
    public static final double HIVE_BLUE_UP_CELL_SIDE = -1.0;  // -Y, far wall

    // ------------------------------------------------------------------
    // FLOWER (manual 9.7)
    // ------------------------------------------------------------------

    /** Top opening 4 in. across, 21.5 in. above the TILEs (checklist 1.7, +/- 0.25 in.). */
    public static final double FLOWER_TOP_OPENING_DIAMETER_MM = 101.5;
    public static final double FLOWER_TOP_HEIGHT_MM = 546.0;

    /** Bottom retrieval opening, 3.55 in. tall by 3.57 in. deep; lower ring hole 2.79 in. */
    public static final double FLOWER_RETRIEVAL_OPENING_HEIGHT_MM = 90.0;
    public static final double FLOWER_RETRIEVAL_OPENING_DEPTH_MM = 91.0;
    public static final double FLOWER_BOTTOM_HOLE_DIAMETER_MM = 71.0;

    /**
     * TODO(robot): standoff of the FLOWER bottom ring from the wall, and the station along the
     * wall. Neither is published; 100 mm clears the ring and the 100 mm of the alliance-half
     * centre is only a starting point. Check both against Field Acceptance Checklist Figure 2.
     */
    public static final double FLOWER_RING_FROM_WALL_MM = 100.0;
    private static final double FLOWER_STATION_X_MM = TILE_MM * 1.5;   // +914.4, centre of a half

    /** Heading points from the field into the FLOWER, i.e. at the wall it is mounted on. */
    public static final Pose2D FLOWER_AUDIENCE_RED = pose(
            +FLOWER_STATION_X_MM, HALF_FIELD_MM - FLOWER_RING_FROM_WALL_MM, Math.PI / 2);
    public static final Pose2D FLOWER_AUDIENCE_BLUE = pose(
            -FLOWER_STATION_X_MM, HALF_FIELD_MM - FLOWER_RING_FROM_WALL_MM, Math.PI / 2);
    public static final Pose2D FLOWER_FAR_RED = pose(
            +FLOWER_STATION_X_MM, -HALF_FIELD_MM + FLOWER_RING_FROM_WALL_MM, -Math.PI / 2);
    public static final Pose2D FLOWER_FAR_BLUE = pose(
            -FLOWER_STATION_X_MM, -HALF_FIELD_MM + FLOWER_RING_FROM_WALL_MM, -Math.PI / 2);

    // ------------------------------------------------------------------
    // Shared types
    // ------------------------------------------------------------------

    /** An axis-aligned rectangle on the TILEs or outside the FIELD, in the field frame. */
    public static final class Zone {
        private final String name;
        private final double minX, maxX, minY, maxY;

        public Zone(String name, double minX, double maxX, double minY, double maxY) {
            this.name = name;
            this.minX = minX;
            this.maxX = maxX;
            this.minY = minY;
            this.maxY = maxY;
        }

        public String getName() {
            return name;
        }

        /** PARK scores when the ROBOT is at least partially in a LOADING ZONE, so test its corners. */
        public boolean contains(double xMm, double yMm) {
            return xMm >= minX && xMm <= maxX && yMm >= minY && yMm <= maxY;
        }

        public boolean contains(Pose2D pose) {
            return contains(pose.getX(DistanceUnit.MM), pose.getY(DistanceUnit.MM));
        }

        public Pose2D centre() {
            return pose((minX + maxX) / 2.0, (minY + maxY) / 2.0, 0);
        }

        public double getMinX() {
            return minX;
        }

        public double getMaxX() {
            return maxX;
        }

        public double getMinY() {
            return minY;
        }

        public double getMaxY() {
            return maxY;
        }

        /** The same zone on the other alliance's side: a half turn about the field centre. */
        public Zone mirrored(String newName) {
            return new Zone(newName, -maxX, -minX, -maxY, -minY);
        }
    }

    // ------------------------------------------------------------------
    // Zones (written in the RED frame, mirrored for BLUE)
    // ------------------------------------------------------------------

    /** Red LOADING ZONE: on TILE A5, against the red wall (setup guide 8.3, checklist 3.2). */
    public static final Zone RED_LOADING_ZONE = new Zone(
            "red loading zone",
            HALF_FIELD_MM - LOADING_ZONE_DEPTH_MM, HALF_FIELD_MM,
            tileRowCentreMm(5) - LOADING_ZONE_WIDTH_MM / 2.0,
            tileRowCentreMm(5) + LOADING_ZONE_WIDTH_MM / 2.0);

    /** Blue LOADING ZONE: on TILE F2 (checklist 3.3). */
    public static final Zone BLUE_LOADING_ZONE = RED_LOADING_ZONE.mirrored("blue loading zone");

    /** Red GARDEN: TILE A1, against the audience wall (checklist 3.5). */
    public static final Zone RED_GARDEN = new Zone(
            "red garden",
            tileColumnCentreMm('A') - GARDEN_WIDTH_MM / 2.0,
            tileColumnCentreMm('A') + GARDEN_WIDTH_MM / 2.0,
            HALF_FIELD_MM - GARDEN_DEPTH_MM, HALF_FIELD_MM);

    /** Blue GARDEN: TILE F6, against the far wall (checklist 3.6). */
    public static final Zone BLUE_GARDEN = RED_GARDEN.mirrored("blue garden");

    /** Red ALLIANCE AREA: outside the red wall, spanning TILE seams 1 to 5. */
    public static final Zone RED_ALLIANCE_AREA = new Zone(
            "red alliance area",
            HALF_FIELD_MM, HALF_FIELD_MM + ALLIANCE_AREA_DEPTH_MM,
            -ALLIANCE_AREA_WIDTH_MM / 2.0, ALLIANCE_AREA_WIDTH_MM / 2.0);

    public static final Zone BLUE_ALLIANCE_AREA = RED_ALLIANCE_AREA.mirrored("blue alliance area");

    // ------------------------------------------------------------------
    // Start poses
    // ------------------------------------------------------------------

    /**
     * TEAM CHOICE, not a rule: G303 only requires the ROBOT to be fully contained in its own
     * alliance's columns (A, B, C for red), touching a perimeter wall, clear of the LOADING ZONE
     * and not touching a FLOWER. This standoff is half your robot's footprint plus clearance, so
     * the robot starts back against the red wall. Measure your robot and adjust.
     */
    public static final double START_STANDOFF_FROM_WALL_MM = 250.0;

    /**
     * Red start: back against the red wall on TILE A2, facing the field centre (-X). Row 2 is
     * deliberately clear of the red LOADING ZONE, which is on row 5.
     */
    public static final Pose2D START_RED = pose(
            HALF_FIELD_MM - START_STANDOFF_FROM_WALL_MM, tileRowCentreMm(2), Math.PI);

    /** The start pose for an alliance. Blue is the red pose turned half a turn about the centre. */
    public static Pose2D startPose(Alliance alliance) {
        return forAlliance(START_RED, alliance);
    }

    // ------------------------------------------------------------------
    // Named landmarks (RED frame)
    // ------------------------------------------------------------------

    private static final Map<String, Pose2D> LANDMARKS;

    static {
        Map<String, Pose2D> map = new LinkedHashMap<>();
        map.put("hive", HIVE_CENTRE);
        map.put("flowerAudienceRed", FLOWER_AUDIENCE_RED);
        map.put("flowerAudienceBlue", FLOWER_AUDIENCE_BLUE);
        map.put("flowerFarRed", FLOWER_FAR_RED);
        map.put("flowerFarBlue", FLOWER_FAR_BLUE);
        map.put("loadingZoneRed", RED_LOADING_ZONE.centre());
        map.put("loadingZoneBlue", BLUE_LOADING_ZONE.centre());
        map.put("gardenRed", RED_GARDEN.centre());
        map.put("gardenBlue", BLUE_GARDEN.centre());
        map.put("startRed", START_RED);
        LANDMARKS = Collections.unmodifiableMap(map);
    }

    /** A named landmark in the RED frame. Throws if the name is unknown, rather than returning 0. */
    public static Pose2D landmark(String name) {
        Pose2D pose = LANDMARKS.get(name);
        if (pose == null) {
            throw new IllegalArgumentException(
                    "Unknown landmark '" + name + "'; known: " + LANDMARKS.keySet());
        }
        return pose;
    }

    /** A named landmark expressed for the given alliance. */
    public static Pose2D landmark(String name, Alliance alliance) {
        return forAlliance(landmark(name), alliance);
    }

    // ------------------------------------------------------------------
    // TILE grid
    // ------------------------------------------------------------------

    /** X of a TILE column's centre. 'A' is against the red wall, 'F' against the blue wall. */
    public static double tileColumnCentreMm(char column) {
        int index = Character.toUpperCase(column) - 'A';
        if (index < 0 || index >= TILES_PER_SIDE) {
            throw new IllegalArgumentException("TILE column must be A-F, got '" + column + "'");
        }
        return HALF_FIELD_MM - TILE_MM * (index + 0.5);
    }

    /** Y of a TILE row's centre. Row 1 is against the audience wall, row 6 against the far wall. */
    public static double tileRowCentreMm(int row) {
        if (row < 1 || row > TILES_PER_SIDE) {
            throw new IllegalArgumentException("TILE row must be 1-6, got " + row);
        }
        return HALF_FIELD_MM - TILE_MM * (row - 0.5);
    }

    /** Centre of a TILE such as "A5" or "F6". */
    public static Pose2D tileCentre(String tile) {
        if (tile == null || tile.length() != 2) {
            throw new IllegalArgumentException("TILE id must look like A5, got '" + tile + "'");
        }
        int row = Character.digit(tile.charAt(1), 10);
        return pose(tileColumnCentreMm(tile.charAt(0)), tileRowCentreMm(row), 0);
    }

    // ------------------------------------------------------------------
    // Frame conversions
    // ------------------------------------------------------------------

    /** True if the point is inside the perimeter. */
    public static boolean insideField(double xMm, double yMm) {
        return Math.abs(xMm) <= HALF_FIELD_MM && Math.abs(yMm) <= HALF_FIELD_MM;
    }

    /** Wraps an angle into (-pi, pi]. */
    public static double wrapRadians(double radians) {
        double angle = radians;
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }

    /**
     * Rotates a field-frame vector into the robot frame.
     *
     * @param fieldX      field +X component (toward the red wall)
     * @param fieldY      field +Y component (toward the audience)
     * @param headingRad  robot heading, counter-clockwise, 0 = field +X
     * @return {forward, right}, both in the robot frame. Right is positive to match the wheel mix.
     *
     * The robot's forward unit vector is (cos, sin) in field coordinates and its right unit vector
     * is (sin, -cos), so this is just the two dot products. The matrix is its own inverse, which is
     * why {@link #robotToField} has the same shape.
     */
    public static double[] fieldToRobot(double fieldX, double fieldY, double headingRad) {
        double cos = Math.cos(headingRad);
        double sin = Math.sin(headingRad);
        return new double[]{
                fieldX * cos + fieldY * sin,   // forward
                fieldX * sin - fieldY * cos    // right
        };
    }

    /** Rotates a robot-frame vector back into the field frame. */
    public static double[] robotToField(double forward, double right, double headingRad) {
        double cos = Math.cos(headingRad);
        double sin = Math.sin(headingRad);
        return new double[]{
                forward * cos + right * sin,   // field X
                forward * sin - right * cos    // field Y
        };
    }

    /**
     * Mirrors a RED-frame pose for the given alliance. BLUE is a half turn about the field centre,
     * because BIOBUZZ puts the two alliances on opposite walls and every zone pairs up that way.
     */
    public static Pose2D forAlliance(Pose2D redPose, Alliance alliance) {
        if (alliance != Alliance.BLUE) {
            return redPose;
        }
        return pose(
                -redPose.getX(DistanceUnit.MM),
                -redPose.getY(DistanceUnit.MM),
                redPose.getHeading(AngleUnit.RADIANS) + Math.PI);
    }

    /** Builds a field pose in millimetres and radians, with the heading wrapped. */
    public static Pose2D pose(double xMm, double yMm, double headingRad) {
        return new Pose2D(DistanceUnit.MM, xMm, yMm, AngleUnit.RADIANS, wrapRadians(headingRad));
    }
}
