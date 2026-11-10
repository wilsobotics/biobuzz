package org.firstinspires.ftc.teamcode.collect;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.perception.Angles;
import org.firstinspires.ftc.teamcode.perception.BallClass;
import org.firstinspires.ftc.teamcode.perception.BallMap;
import org.firstinspires.ftc.teamcode.perception.MapBall;
import org.junit.Before;
import org.junit.Test;

import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class CollectPlannerTest {
    private final FieldFrame frame = new FieldFrame(0);
    private final CollectPlanner planner = new CollectPlanner(frame);
    private final Pose robot = new Pose(20, 20, 0);
    private final Set<Integer> none = Collections.<Integer>emptySet();
    private BallMap map;

    @Before
    public void setUp() {
        CollectPlanner.applyWeaverConfig();
        map = new BallMap();
    }

    private List<MapBall> choose(CollectRequest req, int capacity, Set<Integer> skip) {
        return planner.chooseTargets(robot, map.all(), req, new AllowedArea(req, frame), capacity, skip);
    }

    @Test
    public void onlyRequestedClassesAndRealTargetsAreChosen() {
        MapSeeds.confirmed(map, BallClass.POLLEN, 40.0, 20.0, BallClass.BLUE_NECTAR, 30.0, 30.0);
        MapSeeds.hints(map, BallClass.POLLEN, 30.0, 40.0);
        List<MapBall> t = choose(CollectRequest.builder(Alliance.RED, false).build(), 4, none);
        assertEquals(1, t.size());
        assertEquals(BallClass.POLLEN, t.get(0).ballClass());
    }

    @Test
    public void autoKeepsToOurHalfAndOffTheMidline() {
        MapSeeds.confirmed(map, BallClass.POLLEN, 40.0, 20.0, BallClass.POLLEN, 100.0, 20.0,
                BallClass.POLLEN, 70.0, 20.0);
        List<MapBall> t = choose(CollectRequest.builder(Alliance.RED, true).build(), 4, none);
        assertEquals(1, t.size());
        assertEquals(40, t.get(0).x(), 0.5);
    }

    @Test
    public void capacityAndMaybeTwoLimitTheList() {
        MapSeeds.maybeTwo(map, BallClass.POLLEN, 30.0, 20.0);
        MapSeeds.confirmed(map, BallClass.POLLEN, 40.0, 20.0, BallClass.POLLEN, 45.0, 30.0);
        assertEquals(2, choose(CollectRequest.builder(Alliance.RED, false).build(), 3, none).size());
        assertEquals(1, choose(CollectRequest.builder(Alliance.RED, false).maxTargets(1).build(), 3, none).size());
    }

    @Test
    public void weightsBeatDistance() {
        // pollen scores 10/1 = 10; nectar 25/3 = 8.3, so nectar wins despite being further
        MapSeeds.confirmed(map, BallClass.POLLEN, 30.0, 20.0, BallClass.RED_NECTAR, 45.0, 20.0);
        CollectRequest req = CollectRequest.builder(Alliance.RED, false)
                .classes(BallClass.POLLEN, BallClass.RED_NECTAR).weight(BallClass.RED_NECTAR, 3.0).build();
        assertEquals(BallClass.RED_NECTAR, choose(req, 1, none).get(0).ballClass());
    }

    @Test
    public void skippedTargetsAreLeftOut() {
        MapSeeds.confirmed(map, BallClass.POLLEN, 40.0, 20.0);
        Set<Integer> skip = new HashSet<Integer>();
        skip.add(map.all().get(0).id());
        assertTrue(choose(CollectRequest.builder(Alliance.RED, false).build(), 4, skip).isEmpty());
    }

    @Test
    public void planVisitsTargetsAndProducesAPedroPath() {
        MapSeeds.confirmed(map, BallClass.POLLEN, 40.0, 20.0, BallClass.POLLEN, 30.0, 40.0);
        CollectRequest req = CollectRequest.builder(Alliance.RED, false).build();
        AllowedArea area = new AllowedArea(req, frame);
        List<MapBall> t = planner.chooseTargets(robot, map.all(), req, area, 4, none);
        CollectPlanner.Plan p = planner.plan(robot, t,
                planner.obstacles(robot, area, Collections.<ObstacleProvider>emptyList()));
        assertNotNull(planner.lastError(), p);
        assertNotNull(p.path);
        assertEquals(2, p.targets.size());
        assertTrue(p.preview.size() > 2);
    }

    @Test
    public void planToReachesALookPose() {
        CollectRequest req = CollectRequest.builder(Alliance.RED, false).build();
        AllowedArea area = new AllowedArea(req, frame);
        CollectPlanner.Plan p = planner.planTo(robot, new Pose(36, 108, Math.PI),
                planner.obstacles(robot, area, Collections.<ObstacleProvider>emptyList()));
        assertNotNull(planner.lastError(), p);
        assertTrue(p.targets.isEmpty());
    }

    @Test
    public void obstaclesIncludeOutsideBandsOnlyWhenTheRobotIsInside() {
        CollectRequest req = CollectRequest.builder(Alliance.RED, true).build();
        AllowedArea area = new AllowedArea(req, frame);
        List<ObstacleProvider> none = Collections.<ObstacleProvider>emptyList();
        assertEquals(6, planner.obstacles(robot, area, none).size());
        assertEquals(5, planner.obstacles(new Pose(100, 20, 0), area, none).size());
    }

    @Test
    public void planToTurnsSmoothlyToTheLookHeading() {
        CollectRequest req = CollectRequest.builder(Alliance.RED, false).build();
        AllowedArea area = new AllowedArea(req, frame);
        Pose start = new Pose(20, 20, -Math.PI / 2);
        CollectPlanner.Plan p = planner.planTo(start, new Pose(36, 108, Math.PI),
                planner.obstacles(start, area, Collections.<ObstacleProvider>emptyList()));
        assertNotNull(planner.lastError(), p);
        assertEquals(0, Angles.wrap(p.path.endPose().heading() - Math.PI), 1e-6);
        assertEquals(0, Angles.wrap(p.path.heading(0) - start.heading()), 1e-6);
        double maxStep = 0;
        for (int i = 1; i <= 20; i++) {
            double step = Angles.wrap(p.path.heading(i / 20.0) - p.path.heading((i - 1) / 20.0));
            maxStep = Math.max(maxStep, Math.abs(step));
        }
        assertTrue("max heading step " + maxStep, maxStep < 0.5); // no last-moment hook swing
    }
}
