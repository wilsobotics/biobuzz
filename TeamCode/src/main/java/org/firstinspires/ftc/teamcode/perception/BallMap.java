package org.firstinspires.ftc.teamcode.perception;

import com.pedropathing.math.Pose;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.EnumSet;
import java.util.Iterator;
import java.util.List;
import java.util.Set;

/**
 * Field-frame ball map. Data association is one-to-one per frame: each sighting updates at
 * most one ball and each ball takes at most one sighting, so two balls are never merged.
 * A ball in view but not seen only counts a miss when no nearer ball could be hiding it: neither
 * a sighting of this frame nor a recently seen confirmed map ball of any class (colour pipelines
 * only report one class per frame). A ball occluded by another is never deleted for being "missing".
 *
 * <p>Meant to be used from the robot loop thread. The MapBall objects it returns are live
 * views of the map, so callers must not hold them across loops expecting a frozen snapshot.
 */
public final class BallMap {
    private final ArrayList<MapBall> balls = new ArrayList<MapBall>();
    private int nextId = 1;

    /**
     * Folds one frame's sightings into the map. robot/mount may be null (tests); then the
     * "in view but not seen" removal is skipped.
     */
    public synchronized void update(long nowMs, Pose robot, CameraMount mount, List<Sighting> sightings) {
        update(nowMs, robot, mount, sightings, EnumSet.allOf(BallClass.class));
    }

    /**
     * As above, but only balls whose class is in {@code observable} (the classes the frame's
     * pipeline can see) can accumulate an in-view miss; other classes are left untouched.
     */
    public synchronized void update(long nowMs, Pose robot, CameraMount mount, List<Sighting> sightings,
                                    Set<BallClass> observable) {
        prune(nowMs);
        int existing = balls.size();
        boolean[] ballUsed = new boolean[existing];
        boolean[] sightUsed = new boolean[sightings.size()];

        ArrayList<double[]> pairs = new ArrayList<double[]>();
        for (int si = 0; si < sightings.size(); si++) {
            Sighting s = sightings.get(si);
            for (int bi = 0; bi < existing; bi++) {
                MapBall b = balls.get(bi);
                if (b.ballClass() != s.ballClass) {
                    continue;
                }
                double d = b.distanceTo(s.x, s.y);
                if (d <= gate(s, b)) {
                    pairs.add(new double[]{d, si, bi});
                }
            }
        }
        Collections.sort(pairs, new Comparator<double[]>() {
            @Override
            public int compare(double[] a, double[] b) {
                return Double.compare(a[0], b[0]);
            }
        });
        for (double[] p : pairs) {
            int si = (int) p[1];
            int bi = (int) p[2];
            if (sightUsed[si] || ballUsed[bi]) {
                continue;
            }
            sightUsed[si] = true;
            ballUsed[bi] = true;
            fuse(balls.get(bi), sightings.get(si), nowMs);
        }

        ArrayList<MapBall> gone = new ArrayList<MapBall>();
        if (robot != null && mount != null) {
            for (int bi = 0; bi < existing; bi++) {
                if (ballUsed[bi]) {
                    continue;
                }
                MapBall b = balls.get(bi);
                if (!observable.contains(b.ballClass())) {
                    continue; // this frame's pipeline cannot see this class: no evidence either way
                }
                if (CameraView.canSee(robot, mount, b.x, b.y, b.ballClass().centerHeightIn(),
                        PerceptionConfig.VIEW_REMOVE_RANGE_IN)) {
                    if (hiddenBehindSighting(robot, mount, b, sightings)
                            || hiddenBehindMapBall(robot, mount, b, nowMs)) {
                        continue; // may be hidden behind a nearer ball: no evidence either way
                    }
                    b.missFrames++;
                    if (b.missFrames >= PerceptionConfig.MISS_FRAMES_TO_REMOVE) {
                        gone.add(b);
                    }
                } else {
                    b.missFrames = 0; // misses must be consecutive in-view frames
                }
            }
        }
        balls.removeAll(gone);

        for (int si = 0; si < sightings.size(); si++) {
            if (!sightUsed[si]) {
                balls.add(new MapBall(nextId++, sightings.get(si), nowMs));
            }
        }
        prune(nowMs);
    }

    /**
     * True when a ball seen this frame sits nearer the camera and within one ball diameter's
     * angular width (atan2(diameter, range to that nearer ball)) of b's bearing, so the detector
     * may have seen both as one. Every sighting of the frame counts: one that matched a map ball
     * and one that is about to become a new ball can hide b equally well.
     */
    static boolean hiddenBehindSighting(Pose robot, CameraMount mount, MapBall b, List<Sighting> sightings) {
        double cx = mount.camX(robot);
        double cy = mount.camY(robot);
        double range = Math.hypot(b.x - cx, b.y - cy);
        double bearing = Math.atan2(b.y - cy, b.x - cx);
        for (Sighting s : sightings) {
            if (occludes(cx, cy, range, bearing, s.x, s.y, s.ballClass.diameterIn)) {
                return true;
            }
        }
        return false;
    }

    /**
     * True when a confirmed map ball of any class, seen within OCCLUDER_RECENT_MS, sits nearer the
     * camera on b's bearing. Covers colour-pipeline cycling, where the nearer ball's class cannot
     * appear in this frame's sightings at all.
     */
    private boolean hiddenBehindMapBall(Pose robot, CameraMount mount, MapBall b, long nowMs) {
        double cx = mount.camX(robot);
        double cy = mount.camY(robot);
        double range = Math.hypot(b.x - cx, b.y - cy);
        double bearing = Math.atan2(b.y - cy, b.x - cx);
        for (MapBall o : balls) {
            if (o == b || !o.isConfirmed() || nowMs - o.lastSeenMs > PerceptionConfig.OCCLUDER_RECENT_MS) {
                continue;
            }
            if (occludes(cx, cy, range, bearing, o.x, o.y, o.ballClass().diameterIn)) {
                return true;
            }
        }
        return false;
    }

    /** A ball of diameter d at (ox, oy) is nearer than range and within its angular width of bearing. */
    private static boolean occludes(double cx, double cy, double range, double bearing,
                                    double ox, double oy, double d) {
        double or = Math.hypot(ox - cx, oy - cy);
        if (or >= range || or < 1e-6) {
            return false;
        }
        double width = Math.atan2(d, or);
        return Math.abs(Angles.wrap(Math.atan2(oy - cy, ox - cx) - bearing)) <= width;
    }

    /** Tight gate (below one ball diameter) when either side is precise; wide only when both are not. */
    static double gate(Sighting s, MapBall b) {
        double statistical = Math.max(PerceptionConfig.MIN_GATE_IN,
                PerceptionConfig.GATE_SIGMAS * Math.max(s.sigma, b.sigma));
        boolean precise = s.sigma <= PerceptionConfig.TARGET_SIGMA_IN || b.isPrecise();
        if (!precise) {
            return statistical;
        }
        return Math.min(statistical, PerceptionConfig.TIGHT_GATE_DIAMETERS * s.ballClass.diameterIn);
    }

    private static void fuse(MapBall b, Sighting s, long nowMs) {
        // Compare with the sighting that last set the shape flag, not the fused sigma: fusing
        // shrinks b.sigma, which would otherwise freeze maybeTwo against equal-resolution views.
        double shapeS = Math.max(PerceptionConfig.MIN_SIGMA_IN, s.sigma);
        boolean sharper = shapeS <= b.shapeSigma * PerceptionConfig.SHAPE_SIGMA_SLACK;
        double wb = 1.0 / (b.sigma * b.sigma);
        double sSigma = Math.max(PerceptionConfig.MIN_SIGMA_IN, s.sigma); // sigma 0 would give infinite weight
        double ws = 1.0 / (sSigma * sSigma);
        b.x = (b.x * wb + s.x * ws) / (wb + ws);
        b.y = (b.y * wb + s.y * ws) / (wb + ws);
        b.sigma = Math.max(PerceptionConfig.MIN_SIGMA_IN, Math.sqrt(1.0 / (wb + ws)));
        b.quality = Math.max(b.quality, s.quality);
        if (sharper) {
            b.maybeTwo = s.maybeTwo;
            b.shapeSigma = Math.min(b.shapeSigma, shapeS); // the bar never creeps upward
        }
        b.seenCount++;
        b.lastSeenMs = nowMs;
        b.missFrames = 0;
    }

    /** Drops unconfirmed candidates that were not seen again and balls unseen for FORGET_MS. */
    public synchronized void prune(long nowMs) {
        Iterator<MapBall> it = balls.iterator();
        while (it.hasNext()) {
            MapBall b = it.next();
            long age = nowMs - b.lastSeenMs;
            if ((!b.isConfirmed() && age > PerceptionConfig.CANDIDATE_TIMEOUT_MS)
                    || age > PerceptionConfig.FORGET_MS) {
                it.remove();
            }
        }
    }

    /**
     * The intake just took a ball: remove the one nearest the intake mouth (exactly one).
     * A "maybe two" ball loses its flag first and stays.
     */
    public synchronized MapBall removePickup(Pose robot) {
        double h = robot.heading();
        double mx = robot.x() + PerceptionConfig.INTAKE_REACH_IN * Math.cos(h);
        double my = robot.y() + PerceptionConfig.INTAKE_REACH_IN * Math.sin(h);
        MapBall best = null;
        double bestD = PerceptionConfig.PICKUP_SEARCH_IN;
        for (MapBall b : balls) {
            double d = b.distanceTo(mx, my);
            if (d <= bestD) {
                best = b;
                bestD = d;
            }
        }
        if (best == null) {
            return null;
        }
        if (best.maybeTwo) {
            best.maybeTwo = false;
            return best;
        }
        balls.remove(best);
        return best;
    }

    public synchronized List<MapBall> all() {
        return new ArrayList<MapBall>(balls);
    }

    public synchronized List<MapBall> targets() {
        ArrayList<MapBall> out = new ArrayList<MapBall>();
        for (MapBall b : balls) {
            if (b.isTarget()) {
                out.add(b);
            }
        }
        return out;
    }

    public synchronized List<MapBall> hints() {
        ArrayList<MapBall> out = new ArrayList<MapBall>();
        for (MapBall b : balls) {
            if (b.isHint()) {
                out.add(b);
            }
        }
        return out;
    }

    public synchronized boolean contains(int id) {
        for (MapBall b : balls) {
            if (b.id() == id) {
                return true;
            }
        }
        return false;
    }

    public synchronized int size() {
        return balls.size();
    }

    public synchronized void clear() {
        balls.clear();
    }
}
