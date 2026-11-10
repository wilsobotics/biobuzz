# Ball perception + Weaver collect: design

Status: draft v2 for review (2026-11-09, revised after review; manual TU08). Owner: Tayyab (perception + pathing). Code deadline: Mon 16 Nov 2026.

## 1. Scope

**Ours:** field/ball detection on the Limelight 3A, a field-frame ball map, and a collect
command that plans with Marrow Weaver and drives with Pedro Pathing 3, in auto and as a
TeleOp assist.

**Not ours (we only expose data or hooks for them):** robot detection, hive state, shooter /
turret aiming, localization, the intake mechanism, match flow (when to score).

**Libraries:** NextFTC 1.1.0 (commands/subsystems), Pedro Pathing 3.0.2 (`core` + `revhub`),
Marrow 1.2.5 (Weaver), FTC Dashboard (live tuning + field drawing). NextFTC's Pedro extension
targets Pedro 2, so we write our own follow-path command.

## 2. Decisions (from the design Q&A)

| # | Decision |
|---|----------|
| Camera | One Limelight 3A on the turret, pitched by an ungeared, encoder-less tilt servo. Ball mode = tilted down, goal mode = up. Ball-mode pitch still to be measured (prelim bench). |
| Turret | Held at 0° in ball mode for v1 (180° later for dual intake, as a setting). |
| Output | Field-frame ball positions, using the odometry pose at frame capture time (goBILDA pods + Pinpoint via the Pedro follower). |
| Mode | Camera mode follows a robot task state (COLLECT → ball, SCORE → goal) set explicitly by commands, never inferred from motion. |
| Structure | `CameraSubsystem` is the sole Limelight/servo owner and publishes stamped frames; feature subsystems pull the latest frame by frame id. Logic lives in plain Java classes tested on a laptop. |
| Q6 Targets | Caller picks target classes. **v1 robot is pollen only** (2026-11-10): default = pollen; `CollectConfig.COLLECT_NECTAR` adds own-alliance nectar for later robots. |
| Q7 Re-plan | Event driven with a ~0.3 s cooldown: target gone, better new ball, unexpected or missed pickup, off path (> ~6 in or > ~20°), stalled (~0.5 s little progress), path done but not full. **Not** on every beam break. A target missed twice is skipped. |
| Q8 Full | Collect command ends returning `FULL`; the caller chains scoring. |
| Q9 Obstacles | Static field obstacles as Weaver zones + an `ObstacleProvider` slot (empty by default). No dynamic robot prediction. |
| Collisions | Handled by alliance work zones (partner's zone = obstacle + its balls are not targets) plus the off-path / stall re-plans. |
| Q10 Memory | Ball map keeps balls for the match; unseen > 4 s → dropped; in camera view but not seen → dropped at once. |
| Q11 Zones | Gamepad-selected presets during init (default whole field), preset shapes in `@Config`. |
| Q12 Use | Auto + TeleOp assist (hold button; release or stick input cancels at once). |
| Debug | FTC Dashboard: `@Config` on config classes; field overlay of pose, ball map, path, zones. |

## 3. Assumptions (not yet discussed — object to any)

1. **Pipelines (checked against the Limelight docs):** each Limelight pipeline has one *type*. A **neural detector** pipeline reports every class its model knows in each frame, so balls (all 3 classes) fit in **one** pipeline; AprilTags need a separate **fiducial** pipeline. A **colour** pipeline tracks one HSV range, so a colour-based approach needs one pipeline per colour and must cycle them (each colour then gets about 1/3 of the frames, plus a settle per switch). Design: `CameraConfig` holds a *list* of ball pipelines, each with a type (NEURAL or COLOUR plus its class). One entry = no cycling; several = round-robin, and each frame records which pipeline produced it. Goal tags stay on pipeline 0 (as `TipDetects` uses). Ask Vatsal what colour problem he found.
2. **Turret check:** we don't command the turret; if its angle is exposed and > 2° from the ball-mode angle, ball frames are rejected.
3. **Lens geometry:** offsets from CAD at turret 0°; lens height fixed ~12.4 in until the mount is final.
4. **Pose:** read through a `PoseSource` interface (Pedro follower in production), with 0.5 s of history; capture time = Control Hub receive time − Limelight capture + targeting latency.
5. **Settle:** after a mode change no frames publish until the settle time has passed **and** a result from the new pipeline arrives.
6. **Ball quality instead of a hard confidence cut:** 0.44 was Hive Vision's default for *their* model, so it means nothing for ours. We keep a low floor (0.25, only to drop junk) and give every sighting a **quality** = detector confidence × range factor × frame-edge factor. Map balls get quality from their sightings (more frames and closer views raise it). The real thresholds come from our capture-session data (precision/recall on labelled clips), then live tuning in FTC Dashboard.
7. **Never merge two balls into one (see §5 "Data association"):** one-to-one matching per frame, a match gate smaller than the ball diameter at close range, split on conflict, remove only one ball per pickup, and a "probably 2 balls" flag for wide boxes. Unit tests cover touching pairs.
8. **Ball heights:** per class (pollen 1.4 in, nectar 1.8 in centre).
9. **Far / edge sightings are kept, not ignored:** they get low quality. Balls at or above `TARGET_QUALITY` are targets; lower ones are **hints**. With no targets, the robot investigates the best hint (drives to a pose where it's in good range) rather than scanning blindly; with no hints, it checks known ball sources (our garden, loading zone, flower retrieval openings) before giving up.
10. **Intake status:** the collect command reads an `IntakeStatus` interface (`ballCount()`, `capacity()`, `isFull()`, pickup event). The Intake class was reverted on master, so we don't import it; an adapter is written when it lands. `maxTargets = capacity − count` (and ≤ 7, Weaver's exact-ordering cap).
11. **Weaver geometry:** intake width 11.3 in, robot 18 × 18 in, clearance 2 in; intake faces robot +X (Pedro forward).
12. **Field obstacles (manual TU08 §9, Fig. 9-4/9-5; verify on a real field):** in field inches with x along the audience side (columns A to F, 0 to 144) and y away from the audience (rows 1 to 6):
    - hive frame: about x 47 to 97, y 52.5 to 91.5 (49.46 × 38.95 in base, centred);
    - flowers on the perimeter at about (48, 144), (96, 0), (0, 48), (144, 96), each about 8 in across;
    - gardens are corner strips (red bottom-left, blue top-right), and loading zones are 23 × 11 in on the alliance walls. They are ball sources, not obstacles.

    These map into Pedro's frame through one transform in config.
13. **Timeouts:** collect default 20 s in auto, none in TeleOp (button controls it).
14. **Tests:** plain-Java logic tested with JUnit 4 local unit tests (`./gradlew :TeamCode:testDebugUnitTest`); needs `testImplementation 'junit:junit:4.13.2'`.
15. **Coordinates:** Pedro field frame everywhere (inches, origin at a corner, heading CCW+ radians). The dashboard draw converts to its centre origin.
16. **AUTO side rule (manual G402):** during AUTO, columns A to C are the red side and D to F the blue side. Touching scoring elements on the opponent's side, or contacting their robot there, is a MAJOR FOUL. So in AUTO the opponent's half is **always** an obstacle, and balls within a margin of the midline (4 in + projection error) are not targets. This is enforced in code, not left to the zone preset. In TeleOp only the chosen preset applies.
17. **Ball sources (manual §10):** at the start no balls are loose on the floor. Pollen starts in the flowers (4 each), the gardens (4 in a line) and the pre-loads; nectar starts in the cells and the alliance area and comes in through the loading zone. These are the search spots in 9. Whether our intake can take pollen from a flower retrieval opening is a question for the intake owner.

18. **Strategy is plug-in, not built in.** Match strategy is still being decided, so nothing strategic is hard-coded. Every collect run takes a `CollectRequest`. The AUTO side rule (16) and the field obstacles are the only fixed parts, because they are rules and geometry, not strategy. Defaults allow everything, so the robot is fully capable whatever strategy is picked:

    | Field | What it controls | Default |
    |-------|------------------|---------|
    | `classes` | which balls count as targets | pollen (own nectar too if `COLLECT_NECTAR`) |
    | `classWeights` | per-class priority. Weaver orders by route; weights bias which balls are offered when there are more than `maxTargets` | all 1.0 |
    | `zone` | allowed area (preset or custom polygon) | whole field |
    | `sources` | named ball sources to search, each enable-able: `GARDEN_OWN`, `LOADING_ZONE_OWN`, `FLOWER_1..4` (retrieval opening; approach pose = intake facing the wall), plus custom poses | loading zone (ours); gardens off (they score); flowers off until the intake confirms |
    | `useHints` | investigate far/low-quality sightings | true |
    | `maxTargets` | cap below intake capacity | capacity − count |
    | `timeoutSec` | end with TIMEOUT | 20 auto / none TeleOp |
    | `endWhenFull` | stop at FULL (false = keep going, e.g. an intake that handles extras) | true |

    Ball sources live in one table in config (`BallSources`), so adding a new spot is one line. A flower source is only useful if the intake can pull from the retrieval opening; it's off by default until the intake owner confirms that.
## 4. Architecture

```
Limelight 3A ──► CameraSubsystem ──CameraFrame(id, mode, captureTime, pose, detections)──► BallTrackingSubsystem
   ▲  tilt servo         │                                                                     │ BallMap (field frame)
   │                     └──► (goal-mode frames for other people's code)                       ▼
   │                                                                              CollectCommand
   │  setMode(BALL/GOAL)                                                           ├─ CollectPlanner → Weaver → Pedro Path
   └──────────────────────────────── commands set task state ◄────────────────────├─ ReplanPolicy (events + cooldown)
                                                                                   ├─ PathFollower → PedroPathFollower (Pedro 3 follower)
                                                                                   └─ IntakeStatus (pickups / full)
```

### Packages (TeamCode)

`org.firstinspires.ftc.teamcode.perception`
- `CameraMode` (BALL, GOAL), `PerceptionConfig` (`@Config`, every perception tunable).
- `CameraSubsystem` (NextFTC `Subsystem`): owns `Limelight3A` + tilt servo; `setMode()`, pipeline cycling (`PipelineCycle`, `PipelineSpec`), publishes the latest `CameraFrame`; `recordPose(t, pose)` feeds the pose history.
- `FrameGate` (plain): `RawResult` (plain copy of the `LLResult`) + pose history → `CameraFrame`; applies new/pipeline/staleness/settle/turret gating.
- `PoseSource` (interface), `PoseHistory` (plain ring buffer, interpolates pose at a timestamp).
- `Detection`, `Sighting` (a projected detection with sigma + quality), `BallClass`, `CameraMount`, `CameraView` (is a floor point in view), `Angles`.
- `BallProjector` (plain): tx/ty + camera pose + per-class ball height → field (x, y); ported from Hive Vision `BallMath`.
- `BallMap` / `MapBall` (plain): confirm, one-to-one association, age-out (4 s), visibility- and occlusion-aware removal, `removePickup(pose)` on pickup, `targets()` / `hints()`.
- `BallTrackingSubsystem` (NextFTC `Subsystem`): pulls new frames, feeds `BallMap`.

`org.firstinspires.ftc.teamcode.collect`
- `CollectRequest` (builder, see assumption 18), `BallSource` (source table), `CollectConfig` (`@Config`), `CollectResult` (FULL, NO_TARGETS, TIMEOUT, INTERRUPTED), `Alliance`.
- `FieldRect`, `FieldFrame` (manual ↔ Pedro frame), `ZonePreset`, `AllowedArea` (zone + AUTO half rule).
- `ObstacleProvider` (interface) and `FieldGeometry` (static obstacles, halves).
- `IntakeStatus` (interface).
- `CollectPlanner` (plain): map + pose + zone (+ AUTO half rule) + capacity → Weaver call → Pedro `Path`.
- `SearchPlanner` (plain): with no targets, picks the best hint's viewing pose, then known ball sources; `CollectPlanner.planTo` makes the path to that pose.
- `ReplanPolicy` (plain): decides when to re-plan.
- `CollectBrain` (plain): the collect state machine (plan, follow, re-plan, search, wait, end).
- `PathFollower` (interface) and `PedroPathFollower` (over the Pedro 3 `Follower`).
- `CollectCommand` (NextFTC `Command` around `CollectBrain`): plans, follows, re-plans, ends with a `CollectResult`; `asTeleopAssist(...)` cancels on release/stick; optional requirements.

`org.firstinspires.ftc.teamcode.debug`
- `FieldDrawer`: FTC Dashboard overlay of pose, ball map (colour by class, faded by age), path, zones.

`org.firstinspires.ftc.teamcode.opmodes`
- `CollectTest` (bench/practice TeleOp), `CountingIntakeStatus`, `RobotHooks` (follower factory).

## 5. Behaviour details

**Frame pipeline (each loop).** CameraSubsystem reads `getLatestResult()`; if it is new
(timestamp changed), valid, from the expected pipeline, fresh (staleness ≤ 150 ms) and captured after the
settle time (judged by capture time, not poll time), it looks up the pose at capture time and
publishes a `CameraFrame` with an incrementing id. Poses come from `recordPose()`, which the
OpMode calls right after `follower.update()`; the subsystem's own `periodic()` runs before
that update, so sampling the `PoseSource` there would lag a loop (it is only a fallback).

**Ball map update.** For each detection of a ball class: project to field and compute its quality. Data association then runs in this order:
1. **One-to-one per frame.** Build all (detection, map ball) pairs of the same class within the gate, sort by distance, and assign greedily. Each detection and each map ball is used at most once per frame. Two detections in one frame are therefore never merged, even if their projections are close.
2. **Gate.** `min(0.8 × ball diameter, k × projection error at that range)`, with a floor for noise. Touching pollen (2.8 in apart) at close range fall outside each other's gate. At long range, where we can't separate them anyway, they stay hints until a closer view resolves them.
3. **Unmatched detection** becomes a candidate (pending) ball, confirmed after 2 frames. Candidates follow the same one-to-one rule.
4. **Split on conflict.** If a map ball's best match was taken by another detection while a second, nearby detection is unmatched, the second becomes its own ball. It is not averaged in.
5. **Wide boxes.** A detector box much wider than tall (aspect > 1.6, when corners are available) is flagged "maybe 2". The planner treats it as 2 targets, and a closer view resolves it.
6. **Pickup removes exactly one ball**: the nearest in front of the intake. Hive Vision's 12 in clear-radius would wipe out neighbours.

Then, for every mapped ball inside the current camera footprint (FOV minus margin, within range) that was not matched, count a miss; remove it after MISS_FRAMES_TO_REMOVE (2) consecutive misses, so one dropped detector frame doesn't delete a ball. A ball that lies within one ball diameter's angular width (atan2(diameter, range to the nearer ball)) of a ball seen this frame that is closer to the camera may be hidden behind it, so it neither counts a miss nor resets its count. Misses are counted per class, and only for classes the frame's pipeline can see (a colour pipeline never counts misses for the other colour). Finally drop anything unseen for more than 4 s.

**Planning.** Targets = map balls with quality ≥ `TARGET_QUALITY`, of the requested classes, inside our zone (and our half in AUTO), not on the
skip list, nearest first, capped at `maxTargets`. Obstacles = field obstacles + partner
zone + any `ObstacleProvider`s. Weaver returns an ordered route; `PedroPathingConverter`
turns it into a Pedro `Path`. Targets Weaver can't reach are held out for `UNREACHABLE_RETRY_MS`
(1.5 s), then offered again (the block may be transient). Look paths use a linear heading from the
robot's heading to the look heading, so the robot arrives facing the hint or source.

**No targets.** Go to the viewing pose of the best hint: about 30 in away, facing it, inside our zone. If it confirms there, it becomes a target and we plan normally; if not, it's dropped. With no hints, visit the known ball sources in our zone, nearest first. Each pose is visited once per collect run.

**Re-plan triggers** (ignored inside the cooldown, except path-done and pose jumps, which have
their own minimum intervals: `PATH_DONE_MIN_MS` (100 ms) since the last re-plan, and the cooldown
between pose jumps). A pose jump is a relocalisation: the pose moved more than `POSE_JUMP_IN` at
an implied speed above `POSE_JUMP_IN_PER_S` (120 in/s), so a slow loop is not mistaken for one.
While the intake waits for a pickup at a ball (`PICKUP_GRACE_MS`), path-done, better-ball and
stall are suppressed; the robot is not held, and pose jumps and other events still apply.
- a current target removed from the map;
- a hint becomes a target, or a new confirmed ball that is closer than the next target (by > 6 in);
- a pickup that wasn't the expected next target, or reaching a target with no pickup (missed → skip after 2);
- off path > 6 in, or heading error > 20° that has lasted `OFF_PATH_HEADING_MS` (600 ms; Weaver paths are tangent-heading, so the error right after a re-plan is transient);
- stall: < 1 in progress over 0.5 s while following;
- path finished and not full.

**Waiting.** Right after start, an empty map waits up to `START_DWELL_MS` (400 ms, follower stopped)
for targets before searching, and re-plans as soon as one appears. When the search runs out
while an unreachable hold is still running, the brain waits (follower stopped) until the earliest
hold expires and re-plans, rather than ending.

**Ending.** FULL when `IntakeStatus.isFull()`; NO_TARGETS only after the hints and the ball-source
checks find nothing and no unreachable hold is pending; TIMEOUT; INTERRUPTED on cancel. On end the follower is stopped (breakFollowing).

## 6. Testing

- Laptop unit tests (any test dependencies are fine: JUnit 4 + Marrow on the test classpath) for `PoseHistory`, `BallProjector` (against `BallMath` reference values), `BallMap`, `ReplanPolicy`, `CollectBrain`, `CollectPlanner` (with real Weaver), `FrameGate`, zone filtering.
- On robot: prelim bench (pitch + switch timing), then a collect OpMode on a practice field with FTC Dashboard drawing.

## 7. Dependencies on other people

- Drivetrain owners: Pedro 3 constants + localizer (Pinpoint).
- Vatsal: pipeline numbering, and what colour problem he hit (neural vs colour pipelines for balls).
- Intake owner: an `IntakeStatus` adapter once Intake is back on master.
- Field obstacle rectangles from the manual.

## 8. Out of scope / backlog

Dynamic robot avoidance, AprilTag pose correction, tilt servo feedback, turret 180° mode,
per-phase target priority, far-pollen model tuning.
