package org.firstinspires.ftc.teamcode.perception;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;

import com.pedropathing.math.Pose;

import org.junit.Test;

public class PoseHistoryTest {
    @Test
    public void interpolatesBetweenSamples() {
        PoseHistory h = new PoseHistory(500);
        h.add(0, new Pose(0, 0, 0));
        h.add(100, new Pose(10, 4, 0));
        Pose p = h.at(50);
        assertEquals(5, p.x(), 1e-9);
        assertEquals(2, p.y(), 1e-9);
    }

    @Test
    public void headingTakesTheShortWayRound() {
        PoseHistory h = new PoseHistory(500);
        h.add(0, new Pose(0, 0, Math.toRadians(170)));
        h.add(100, new Pose(0, 0, Math.toRadians(-170)));
        assertEquals(0, Angles.wrap(h.at(50).heading() - Math.PI), 1e-9);
    }

    @Test
    public void laterThanNewestReturnsNewest() {
        PoseHistory h = new PoseHistory(500);
        h.add(0, new Pose(0, 0, 0));
        h.add(100, new Pose(10, 0, 0));
        assertEquals(10, h.at(2000).x(), 1e-9);
    }

    @Test
    public void olderThanTheWindowIsNull() {
        PoseHistory h = new PoseHistory(500);
        for (long t = 0; t <= 1000; t += 100) {
            h.add(t, new Pose(t / 10.0, 0, 0));
        }
        assertNull(h.at(300));
        assertEquals(50, h.at(500).x(), 1e-9);
    }

    @Test
    public void emptyHistoryIsNull() {
        assertNull(new PoseHistory(500).at(0));
    }
}
