package org.firstinspires.ftc.teamcode.collect;

import com.skeletonarmy.marrow.zones.Zone;

import java.util.List;

/** Extra Weaver obstacles, read at every re-plan (e.g. robots from someone's robot tracker). Pedro frame. */
public interface ObstacleProvider {
    List<Zone> obstacles();
}
