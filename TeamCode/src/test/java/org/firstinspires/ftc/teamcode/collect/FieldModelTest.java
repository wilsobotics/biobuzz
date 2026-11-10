package org.firstinspires.ftc.teamcode.collect;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.perception.BallClass;
import org.junit.Test;

import java.util.List;

public class FieldModelTest {
    @Test
    public void rectIntersectAndComplement() {
        FieldRect half = new FieldRect(0, 0, 72, 144);
        assertNull(half.intersect(new FieldRect(80, 0, 144, 144)));
        assertEquals(36, half.intersect(new FieldRect(36, 0, 144, 144)).minX, 1e-9);
        List<FieldRect> rest = half.complementIn(FieldRect.FIELD);
        assertEquals(1, rest.size());
        assertEquals(72, rest.get(0).minX, 1e-9);
        assertEquals(4, new FieldRect(10, 10, 20, 20).complementIn(FieldRect.FIELD).size());
    }

    @Test
    public void fieldFrameRotatesAboutTheCentreAndRoundTrips() {
        FieldFrame id = new FieldFrame(0);
        assertEquals(10, id.toPedro(10, 20)[0], 1e-9);
        FieldFrame q = new FieldFrame(90);
        double[] p = q.toPedro(144, 72);
        assertEquals(72, p[0], 1e-9);
        assertEquals(144, p[1], 1e-9);
        double[] back = q.toManual(p[0], p[1]);
        assertEquals(144, back[0], 1e-9);
        assertEquals(72, back[1], 1e-9);
        assertEquals(Math.PI / 2, q.headingToPedro(0), 1e-9);
    }

    @Test
    public void staticObstaclesAreTheHiveAndFourFlowers() {
        assertEquals(5, FieldGeometry.staticObstacles(new FieldFrame(0)).size());
    }

    @Test
    public void defaultRequestIsPollenOnlyWithSafeSources() {
        CollectRequest r = CollectRequest.builder(Alliance.BLUE, false).build();
        assertTrue(r.classes.contains(BallClass.POLLEN));
        assertFalse(r.classes.contains(BallClass.BLUE_NECTAR));   // v1 robot is pollen only
        assertFalse(r.classes.contains(BallClass.RED_NECTAR));
        assertTrue(r.sources.contains("LOADING_ZONE_BLUE"));
        assertFalse(r.sources.contains("LOADING_ZONE_RED"));
        assertFalse(r.sources.contains("GARDEN_BLUE"));   // gardens score: off by default
        assertFalse(r.sources.contains("FLOWER_1"));      // until the intake can use retrieval openings
        assertEquals(0, r.timeoutSec, 1e-9);
        assertEquals(CollectConfig.AUTO_TIMEOUT_SEC, CollectRequest.builder(Alliance.BLUE, true).build().timeoutSec, 1e-9);
        assertEquals(1.0, r.weight(BallClass.POLLEN), 1e-9);
    }

    @Test
    public void nectarFlagAddsOnlyOwnNectar() {
        boolean saved = CollectConfig.COLLECT_NECTAR;
        try {
            CollectConfig.COLLECT_NECTAR = true;
            CollectRequest r = CollectRequest.builder(Alliance.BLUE, false).build();
            assertTrue(r.classes.contains(BallClass.POLLEN));
            assertTrue(r.classes.contains(BallClass.BLUE_NECTAR));
            assertFalse(r.classes.contains(BallClass.RED_NECTAR));
        } finally {
            CollectConfig.COLLECT_NECTAR = saved;
        }
    }

    @Test
    public void autoRestrictsToOwnHalfWithAMidlineMargin() {
        CollectRequest r = CollectRequest.builder(Alliance.RED, true).build();
        AllowedArea a = new AllowedArea(r, new FieldFrame(0));
        assertTrue(a.containsPedro(30, 30));
        assertFalse(a.containsPedro(100, 30));
        assertEquals(1, a.outsideZones().size());
    }

    @Test
    public void teleopWholeFieldHasNoOutsideZones() {
        AllowedArea a = new AllowedArea(CollectRequest.builder(Alliance.RED, false).build(), new FieldFrame(0));
        assertTrue(a.containsPedro(100, 30));
        assertEquals(0, a.outsideZones().size());
    }

    @Test
    public void presetsCycle() {
        assertEquals(ZonePreset.OWN_HALF, ZonePreset.WHOLE_FIELD.next());
        assertEquals(ZonePreset.WHOLE_FIELD, ZonePreset.CUSTOM.next());
        assertEquals(72, ZonePreset.OWN_HALF.rect(Alliance.BLUE).minX, 1e-9);
    }

    @Test
    public void customZoneWithSwappedOrOutOfFieldBoundsIsStillValid() {
        double a = CollectConfig.CUSTOM_MIN_X, b = CollectConfig.CUSTOM_MIN_Y;
        double c = CollectConfig.CUSTOM_MAX_X, d = CollectConfig.CUSTOM_MAX_Y;
        try {
            CollectConfig.CUSTOM_MIN_X = 100;
            CollectConfig.CUSTOM_MAX_X = 20;
            CollectConfig.CUSTOM_MIN_Y = 200;
            CollectConfig.CUSTOM_MAX_Y = -10;
            FieldRect r = ZonePreset.CUSTOM.rect(Alliance.RED);
            assertEquals(20, r.minX, 1e-9);
            assertEquals(100, r.maxX, 1e-9);
            assertEquals(0, r.minY, 1e-9);
            assertEquals(144, r.maxY, 1e-9);
        } finally {
            CollectConfig.CUSTOM_MIN_X = a;
            CollectConfig.CUSTOM_MIN_Y = b;
            CollectConfig.CUSTOM_MAX_X = c;
            CollectConfig.CUSTOM_MAX_Y = d;
        }
    }
}
