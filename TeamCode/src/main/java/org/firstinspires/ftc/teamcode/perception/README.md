# Perception + collect

Ball detection on the Limelight 3A → field-frame ball map → Weaver/Pedro collect command.
Design: `docs/superpowers/specs/2026-11-09-perception-collect-design.md`.

## Wiring it into an OpMode

```java
CameraSubsystem camera = new CameraSubsystem();
BallTrackingSubsystem tracking = new BallTrackingSubsystem(camera);
addComponents(new SubsystemComponent(camera, tracking));          // constructor
// every loop, init and run (onWaitForStart + onUpdate): update the follower, then stamp the pose
follower.update();
camera.recordPose(System.currentTimeMillis(), follower.pose());
// camera.setPoseSource(() -> follower.pose()) still works as a fallback, but periodic() runs
// before follower.update(), so those poses lag a loop. Once recordPose() is called it is used instead.

// AUTO: collect, then the caller decides what's next
CollectCommand collect = new CollectCommand(camera, tracking, new PedroPathFollower(follower),
        intakeStatus, CollectRequest.builder(Alliance.RED, true).build(), Collections.emptyList());
collect.then(camera.modeCommand(CameraMode.GOAL) /* , your score command */).schedule();

// TeleOp: hold-to-collect. asTeleopAssist() only adds the cancel condition (bumper released or a
// stick moved); it does not watch the bumper for you. So run this block on each bumper RISING EDGE,
// building a fresh CollectCommand per press (a command instance is one run), exactly as
// opmodes/CollectTest.onUpdate() does. Pass a requirement shared with your drive command (e.g. the
// drivetrain subsystem) so collect and the TeleOp drive never fight over the follower.
if (gamepad1.right_bumper && !lastBumper) {
    new CollectCommand(camera, tracking, pathFollower, intakeStatus, request, obstacles, drivetrain)
            .asTeleopAssist(() -> gamepad1.right_bumper, () -> stickMagnitude).schedule();
}
lastBumper = gamepad1.right_bumper;
```

## Strategy knobs (CollectRequest)

classes, per-class weights, zone preset, ball sources (garden / loading zone / flowers),
hints on/off, max targets, timeout, end-when-full. AUTO always keeps to our half (manual G402).

## Tuning

All values are in `PerceptionConfig` and `CollectConfig`, live in FTC Dashboard (192.168.43.1:8080/dash).
Set `BALL_PITCH_DOWN_DEG`, `TILT_*_POS` and settle times from the prelim bench before trusting positions.
The bench is the **Limelight Prelim Bench** TeleOp (`opmodes/LimelightPrelimBench.java`): it moves the tilt
servo, shows the measured distance of each ball, times ball/goal switches (use the median for `MODE_SETTLE_MS`),
saves setpoints into `PerceptionConfig` with X (live until the app restarts, so copy the logged `SAVE` values
into `PerceptionConfig.java`), and logs everything to `FIRST/perception/prelim_*.csv` on the Control Hub.

## Behaviour notes

- The map uses the tight gate when either the sighting or the ball is precise, so far, blurry sightings can't absorb a nearby ball.
- "In view but unseen" misses must be consecutive; one dropped detector frame never removes a ball.
- A ball hidden behind a nearer ball (within atan2(diameter, range to the nearer ball) of its bearing) neither counts a miss nor resets it, so two balls in line are never deleted for being merged into one detection. The occluder can be a sighting in this frame or a confirmed map ball of any class seen within `OCCLUDER_RECENT_MS` (colour pipelines can't see other colours).
- A "maybe two" flag is only cleared by a sighting at least as sharp as the one that set it, so a blurry view can never turn two balls into one.
- Frames are settled by capture time, not poll time; a repeat poll of a rejected result keeps its reason.
- A frame waits (POSE_PENDING, up to `POSE_WAIT_MAX_MS`) until the pose history covers its capture time, and ball frames captured before the turret became ready are rejected.
- In AUTO the opponent-half barrier is always an obstacle while the robot is on its own half, even when the robot is outside the chosen zone preset. Look poses are kept clear of field obstacles.
- A search spot counts as visited only after the robot has dwelt there facing it; recovery re-plans on the way go back to the same spot. If the robot is already there but facing away it turns in place (`LOOK_HEADING_TOL_DEG`, `LOOK_TURN_TIMEOUT_MS`).
- Misses only count for classes the frame's pipeline can see, so a colour pipeline never removes balls of the other colour.
- The map prunes stale balls before associating new sightings.
- At a ball, `PICKUP_GRACE_MS` suppresses PATH_DONE, BETTER_BALL and stall re-plans while the pickup lands (the robot is not held; pose jumps and other events still re-plan). A ball missed twice is skipped.
- Targets Weaver can't reach are held out for `UNREACHABLE_RETRY_MS`, then tried again. NO_TARGETS never ends a run while such a hold is pending: the brain waits (stopped) for the earliest hold to expire and re-plans.
- POSE_JUMP is speed-based: more than `POSE_JUMP_IN` at above `POSE_JUMP_IN_PER_S`, at most once per `REPLAN_COOLDOWN_MS`.
- PATH_DONE re-plans at most once per `PATH_DONE_MIN_MS`, so a follower left in manual mode can't re-plan every loop.
- The heading part of OFF_PATH needs the error to last `OFF_PATH_HEADING_MS` (Weaver paths are tangent-heading).
- After start, an empty map waits `START_DWELL_MS` for targets before searching or ending NO_TARGETS.
- Look paths turn linearly from the current heading to the look heading, so the robot arrives facing the hint/source.

## Tests

`./gradlew :TeamCode:testDebugUnitTest` (needs `local.properties` with `sdk.dir`).

## Integration points owned by others

- `opmodes/RobotHooks.createFollower` — drivetrain team's Pedro 3 Follower.
- `collect/IntakeStatus` — intake team's adapter (`ballCount`, `capacity`).
- `PerceptionConfig.GOAL_PIPELINE` / `BALL_PIPELINES` — agree numbering with Vatsal.
