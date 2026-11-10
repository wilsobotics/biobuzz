package org.firstinspires.ftc.teamcode.collect;

import com.pedropathing.math.Pose;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.Zone;

import org.firstinspires.ftc.teamcode.perception.Angles;
import org.firstinspires.ftc.teamcode.perception.BallMap;
import org.firstinspires.ftc.teamcode.perception.MapBall;
import org.firstinspires.ftc.teamcode.perception.PerceptionConfig;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

/** Plain-Java collect loop: plan, follow, re-plan on events, search when empty, end with a result. */
public final class CollectBrain {
    /** WAIT: stopped, nothing to drive to yet (start dwell or an unreachable hold still pending). */
    private enum State { IDLE, TARGETS, LOOK, WAIT, DONE }

    private final CollectRequest req;
    private final BallMap map;
    private final PathFollower follower;
    private final IntakeStatus intake;
    private final CollectPlanner planner;
    private final SearchPlanner search;
    private final AllowedArea area;
    private final List<ObstacleProvider> providers;
    private final ReplanPolicy policy = new ReplanPolicy();
    private final ReplanPolicy.Inputs in = new ReplanPolicy.Inputs();
    private final Set<Integer> skip = new HashSet<Integer>();
    private final Map<Integer, Long> unreachableUntil = new HashMap<Integer, Long>();
    private final Map<Integer, Integer> misses = new HashMap<Integer, Integer>();
    private final Set<String> visitedLooks = new HashSet<String>();

    private State state = State.IDLE;
    private CollectResult result;
    private List<MapBall> planned = Collections.emptyList();
    private List<Point> preview = Collections.emptyList();
    private long startMs;
    private long nowMs;
    private long lookArrivedMs = -1;
    private long waitUntilMs = -1;
    private long lastPoseMs;
    private long reachedMs = -1;
    private int awaitingId = -1;
    private int lastCount;
    private Pose lastPose;
    private boolean pendingUnexpected;
    private boolean pendingMissed;
    private ReplanPolicy.Reason lastReason = ReplanPolicy.Reason.NONE;
    private int replanCount;

    public CollectBrain(CollectRequest req, BallMap map, PathFollower follower, IntakeStatus intake,
                        FieldFrame frame, List<ObstacleProvider> providers) {
        this.req = req;
        this.map = map;
        this.follower = follower;
        this.intake = intake;
        this.planner = new CollectPlanner(frame);
        this.search = new SearchPlanner(frame);
        this.area = new AllowedArea(req, frame);
        this.providers = providers;
        CollectPlanner.applyWeaverConfig();
    }

    public void start(long nowMs) {
        this.nowMs = nowMs;
        startMs = nowMs;
        lastCount = intake.ballCount();
        lastPose = follower.pose();
        lastPoseMs = nowMs;
        policy.reset(nowMs);
        replan(nowMs);
    }

    public void update(long nowMs) {
        if (state == State.IDLE || state == State.DONE) {
            return;
        }
        this.nowMs = nowMs;
        Pose pose = follower.pose();
        boolean jump = isPoseJump(pose, nowMs);
        lastPose = pose;
        lastPoseMs = nowMs;

        if (req.timeoutSec > 0 && nowMs - startMs >= (long) (req.timeoutSec * 1000)) {
            finish(CollectResult.TIMEOUT);
            return;
        }
        trackPickups(pose);
        if (req.endWhenFull && intake.isFull()) {
            finish(CollectResult.FULL);
            return;
        }
        if (state == State.WAIT) {
            if (nowMs >= waitUntilMs || hasTargets(pose)) {
                policy.reset(nowMs);
                replan(nowMs);
            }
            return;
        }
        if (state == State.TARGETS) {
            trackMisses(nowMs, pose);
        }
        // Sitting at the ball: let the pickup land before the path ending or a nearer ball re-plans.
        boolean inGrace = state == State.TARGETS && awaitingId >= 0
                && nowMs - reachedMs < CollectConfig.PICKUP_GRACE_MS;
        boolean pathDone = !follower.isBusy();
        if (state == State.LOOK && pathDone && !jump) {
            if (lookArrivedMs < 0) {
                lookArrivedMs = nowMs;
            }
            if (nowMs - lookArrivedMs < CollectConfig.LOOK_DWELL_MS && !hasTargets(pose)) {
                return;
            }
        }

        in.poseJump = jump;
        in.pathDone = pathDone && !inGrace;
        in.following = !pathDone && !inGrace;
        in.pose = pose;
        in.targetGone = state == State.TARGETS && anyPlannedGone();
        in.unexpectedPickup = pendingUnexpected;
        in.missedTarget = pendingMissed;
        in.betterBall = !inGrace
                && (state == State.LOOK ? hasTargets(pose) : betterBallAvailable(pose));
        Pose closest = pathDone ? null : follower.closestPose();
        in.offPathIn = closest == null ? 0 : Math.hypot(pose.x() - closest.x(), pose.y() - closest.y());
        in.headingErrDeg = closest == null ? 0
                : Math.abs(Math.toDegrees(Angles.wrap(pose.heading() - closest.heading())));

        ReplanPolicy.Reason r = policy.check(nowMs, in);
        if (r != ReplanPolicy.Reason.NONE) {
            lastReason = r;
            replan(nowMs);
        }
    }

    /**
     * A relocalisation, not driving: the implied speed since the last update is above
     * POSE_JUMP_IN_PER_S and the move is above POSE_JUMP_IN. Speed-based so a slow loop
     * (e.g. a long Weaver re-plan) does not look like a jump.
     */
    private boolean isPoseJump(Pose pose, long nowMs) {
        long dt = nowMs - lastPoseMs;
        if (lastPose == null || dt <= 0) {
            return false;
        }
        double dist = Math.hypot(pose.x() - lastPose.x(), pose.y() - lastPose.y());
        return dist > CollectConfig.POSE_JUMP_IN && dist / (dt / 1000.0) > CollectConfig.POSE_JUMP_IN_PER_S;
    }

    public void stop() {
        finish(CollectResult.INTERRUPTED);
    }

    private void replan(long nowMs) {
        replanCount++;
        pendingUnexpected = false;
        pendingMissed = false;
        awaitingId = -1;
        lookArrivedMs = -1;
        Pose pose = follower.pose();
        if (req.endWhenFull && capacityLeft() <= 0) {
            finish(CollectResult.FULL);
            return;
        }
        List<Zone> obstacles = planner.obstacles(pose, area, providers);
        List<MapBall> targets = candidates(pose);
        if (!targets.isEmpty()) {
            CollectPlanner.Plan p = planner.plan(pose, targets, obstacles);
            for (MapBall b : targets) {
                if (p == null || !p.targets.contains(b)) {
                    // Weaver could not reach it: hold it out for a while (may be transient), don't thrash.
                    unreachableUntil.put(b.id(), nowMs + CollectConfig.UNREACHABLE_RETRY_MS);
                }
            }
            if (p != null) {
                planned = new ArrayList<MapBall>(p.targets);
                preview = p.preview;
                follower.follow(p.path);
                state = State.TARGETS;
                return;
            }
        }
        planned = Collections.emptyList();
        if (targets.isEmpty() && nowMs < startMs + CollectConfig.START_DWELL_MS) {
            // Just started on an empty map: give perception a moment before searching or giving up.
            waitUntil(startMs + CollectConfig.START_DWELL_MS);
            return;
        }
        while (true) {
            SearchPlanner.Look look = search.nextLook(pose, map.hints(), req, area, visitedLooks);
            if (look == null) {
                long hold = earliestPendingHold();
                if (hold >= 0) {
                    waitUntil(hold);   // a held ball may become reachable: don't give up yet
                } else {
                    finish(CollectResult.NO_TARGETS);
                }
                return;
            }
            visitedLooks.add(look.key);
            state = State.LOOK;
            if (Math.hypot(look.pose.x() - pose.x(), look.pose.y() - pose.y()) < 3.0) {
                follower.stop();
                lookArrivedMs = nowMs;
                return;
            }
            CollectPlanner.Plan p = planner.planTo(pose, look.pose, obstacles);
            if (p != null) {
                preview = p.preview;
                follower.follow(p.path);
                return;
            }
        }
    }

    private void waitUntil(long untilMs) {
        state = State.WAIT;
        waitUntilMs = untilMs;
        preview = Collections.emptyList();
        follower.stop();
    }

    /** Earliest expiry among unreachable holds still running, or -1 if none. */
    private long earliestPendingHold() {
        long best = -1;
        for (Map.Entry<Integer, Long> e : unreachableUntil.entrySet()) {
            long until = e.getValue();
            // a hold on a ball that has left the map can't become reachable: ignore it
            if (until > nowMs && map.contains(e.getKey()) && (best < 0 || until < best)) {
                best = until;
            }
        }
        return best;
    }

    private void trackPickups(Pose pose) {
        int count = intake.ballCount();
        if (count < lastCount) {
            lastCount = count;     // balls were shot/ejected
            return;
        }
        while (lastCount < count) {
            lastCount++;
            MapBall expected = nextPlanned();
            MapBall removed = map.removePickup(pose);
            boolean asPlanned = removed != null && expected != null && removed.id() == expected.id();
            if (removed != null && removed.id() == awaitingId) {
                awaitingId = -1;
            }
            if (asPlanned) {
                ArrayList<MapBall> rest = new ArrayList<MapBall>(planned);
                rest.remove(expected);
                planned = rest;
            } else {
                pendingUnexpected = true;
            }
        }
    }

    private void trackMisses(long nowMs, Pose pose) {
        if (awaitingId >= 0 && !map.contains(awaitingId)) {
            awaitingId = -1;       // the ball vanished (picked up or dropped): not a miss
        }
        MapBall next = nextPlanned();
        if (awaitingId < 0) {
            if (next != null && mouthDistance(pose, next) <= CollectConfig.REACH_TOL_IN) {
                awaitingId = next.id();
                reachedMs = nowMs;
            }
            return;
        }
        if (nowMs - reachedMs < CollectConfig.PICKUP_GRACE_MS) {
            return;
        }
        Integer prev = misses.get(awaitingId);
        int n = prev == null ? 1 : prev + 1;
        misses.put(awaitingId, n);
        if (n >= CollectConfig.MISS_LIMIT) {
            skip.add(awaitingId);
        }
        awaitingId = -1;
        pendingMissed = true;
    }

    private static double mouthDistance(Pose pose, MapBall b) {
        double h = pose.heading();
        return b.distanceTo(pose.x() + PerceptionConfig.INTAKE_REACH_IN * Math.cos(h),
                pose.y() + PerceptionConfig.INTAKE_REACH_IN * Math.sin(h));
    }

    private MapBall nextPlanned() {
        for (MapBall b : planned) {
            if (map.contains(b.id()) && !skip.contains(b.id())) {
                return b;
            }
        }
        return null;
    }

    private boolean anyPlannedGone() {
        for (MapBall b : planned) {
            if (!map.contains(b.id())) {
                return true;
            }
        }
        return false;
    }

    private boolean isPlanned(int id) {
        for (MapBall b : planned) {
            if (b.id() == id) {
                return true;
            }
        }
        return false;
    }

    private int capacityLeft() {
        return intake.capacity() - intake.ballCount();
    }

    private List<MapBall> candidates(Pose pose) {
        int cap = req.endWhenFull ? capacityLeft() : Integer.MAX_VALUE;
        return planner.chooseTargets(pose, map.all(), req, area, cap, excluded());
    }

    /** Permanently skipped ids plus balls currently held out as unreachable. */
    private Set<Integer> excluded() {
        Set<Integer> out = new HashSet<Integer>(skip);
        for (Map.Entry<Integer, Long> e : unreachableUntil.entrySet()) {
            if (nowMs < e.getValue()) {
                out.add(e.getKey());
            }
        }
        return out;
    }

    private boolean hasTargets(Pose pose) {
        return !candidates(pose).isEmpty();
    }

    private boolean betterBallAvailable(Pose pose) {
        MapBall next = nextPlanned();
        if (next == null) {
            return false;
        }
        double nextD = next.distanceTo(pose.x(), pose.y());
        for (MapBall b : candidates(pose)) {
            if (!isPlanned(b.id())
                    && b.distanceTo(pose.x(), pose.y()) + CollectConfig.BETTER_BALL_MARGIN_IN < nextD) {
                return true;
            }
        }
        return false;
    }

    private void finish(CollectResult r) {
        if (state == State.DONE) {
            return;
        }
        state = State.DONE;
        result = r;
        follower.stop();
    }

    public boolean isDone() { return state == State.DONE; }
    public CollectResult result() { return result; }
    public List<MapBall> plannedTargets() { return planned; }
    public List<Point> preview() { return preview; }
    public ReplanPolicy.Reason lastReason() { return lastReason; }
    public int replanCount() { return replanCount; }
    public Set<Integer> skipped() { return Collections.unmodifiableSet(skip); }
    public String state() { return state.name(); }
    public AllowedArea area() { return area; }
}
