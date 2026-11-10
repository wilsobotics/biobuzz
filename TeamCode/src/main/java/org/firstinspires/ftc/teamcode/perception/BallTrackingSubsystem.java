package org.firstinspires.ftc.teamcode.perception;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import java.util.Set;

import dev.nextftc.core.subsystems.Subsystem;

/** Folds each new BALL-mode frame into the ball map; ages the map every loop. */
public final class BallTrackingSubsystem implements Subsystem {
    private final CameraSubsystem camera;
    private final BallMap map = new BallMap();
    private long lastFrameId = -1;

    public BallTrackingSubsystem(CameraSubsystem camera) {
        this.camera = camera;
    }

    @Override
    public void initialize() {
        map.clear();
        lastFrameId = -1;
    }

    @Override
    public void periodic() {
        long now = System.currentTimeMillis();
        CameraFrame f = camera.latestFrame();
        if (f != null && f.id != lastFrameId && f.mode == CameraMode.BALL) {
            lastFrameId = f.id;
            CameraMount m = CameraMount.fromConfig();
            Set<BallClass> observable = (f.pipeline != null && f.pipeline.type == PipelineSpec.Type.COLOUR)
                    ? EnumSet.of(f.pipeline.colourClass)
                    : EnumSet.allOf(BallClass.class);
            map.update(now, f.robotPose, m, sightings(f, m), observable);
        }
        map.prune(now);
    }

    static List<Sighting> sightings(CameraFrame f, CameraMount m) {
        ArrayList<Sighting> out = new ArrayList<Sighting>();
        for (Detection d : f.balls) {
            Sighting s = Sighting.from(d, f.robotPose, m);
            if (s != null) {
                out.add(s);
            }
        }
        return out;
    }

    public BallMap map() {
        return map;
    }
}
