package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.pedropathing.math.Pose;
import com.skeletonarmy.marrow.weaver.PathResult;
import com.skeletonarmy.marrow.weaver.PedroPathingConverter;
import com.skeletonarmy.marrow.weaver.Weaver;
import com.skeletonarmy.marrow.zones.Point;

import org.junit.Test;

public class SmokeTest {
    @Test
    public void librariesLoadOnTheLaptop() {
        PathResult r = Weaver.builder()
                .start(new Point(10, 10, 0))
                .targets(new Point(40, 40), new Point(60, 20))
                .build();
        assertEquals(2, r.getVisitedTargets().size());
        assertNotNull(PedroPathingConverter.toPath(r));
        assertEquals(3.0, new Pose(3, 4, 0).x(), 1e-9);
        assertNotNull(new Canvas().strokeCircle(0, 0, 1));
    }
}
