package org.firstinspires.ftc.teamcode.collect;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.perception.BallClass;
import org.firstinspires.ftc.teamcode.perception.BallMap;
import org.junit.Test;

import java.util.HashSet;
import java.util.Set;

public class SearchPlannerTest {
    private final FieldFrame frame = new FieldFrame(0);
    private final SearchPlanner search = new SearchPlanner(frame);
    private final Pose robot = new Pose(20, 20, 0);

    private SearchPlanner.Look look(CollectRequest req, BallMap map, Set<String> visited) {
        return search.nextLook(robot, map.hints(), req, new AllowedArea(req, frame), visited);
    }

    @Test
    public void bestHintIsVisitedFromLookDistance() {
        BallMap map = new BallMap();
        MapSeeds.hints(map, BallClass.POLLEN, 100.0, 20.0);
        SearchPlanner.Look l = look(CollectRequest.builder(Alliance.RED, false).build(), map, new HashSet<String>());
        assertEquals(70, l.pose.x(), 1e-6);
        assertEquals(20, l.pose.y(), 1e-6);
        assertEquals(0, l.pose.heading(), 1e-6);
        assertEquals("hint:" + map.all().get(0).id(), l.key);
    }

    @Test
    public void hintsCanBeSwitchedOff() {
        BallMap map = new BallMap();
        MapSeeds.hints(map, BallClass.POLLEN, 100.0, 20.0);
        CollectRequest req = CollectRequest.builder(Alliance.RED, false).useHints(false).onlySources().build();
        assertNull(look(req, map, new HashSet<String>()));
    }

    @Test
    public void thenNearestEnabledSourceOnce() {
        CollectRequest req = CollectRequest.builder(Alliance.RED, false).build();
        Set<String> visited = new HashSet<String>();
        SearchPlanner.Look l = look(req, new BallMap(), visited);
        assertEquals("LOADING_ZONE_RED", l.key);
        assertEquals(36, l.pose.x(), 1e-6);
        assertEquals(Math.PI, Math.abs(l.pose.heading()), 1e-6);
        visited.add(l.key);
        assertNull(look(req, new BallMap(), visited));
    }

    @Test
    public void sourcesOutsideTheAreaAreSkipped() {
        CollectRequest req = CollectRequest.builder(Alliance.RED, false).zone(ZonePreset.NEAR_HALF).build();
        assertNull(look(req, new BallMap(), new HashSet<String>()));   // red loading zone is at y 108
    }
}
