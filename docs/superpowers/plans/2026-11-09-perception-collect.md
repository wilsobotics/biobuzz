# Ball Perception + Weaver Collect Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Turn Limelight 3A ball detections into a field-frame ball map and a NextFTC collect command that plans with Marrow Weaver and drives with Pedro Pathing 3, in auto and as a TeleOp hold-to-collect assist.

**Architecture:** `CameraSubsystem` is the only owner of the Limelight and tilt servo and publishes pose-stamped frames. `BallTrackingSubsystem` folds frames into a `BallMap`, using one-to-one data association so two balls are never merged. `CollectCommand` is a thin NextFTC wrapper around `CollectBrain`, a plain-Java state machine that chooses targets, calls Weaver, follows the Pedro path, re-plans on events and searches hints and ball sources when it has no targets. Every decision lives in plain classes that are unit-tested on a laptop.

**Tech Stack:** Java 8 (TeamCode compiles with `sourceCompatibility 1.8`), FTC SDK 12.0, NextFTC 1.1.0, Pedro Pathing 3.0.2 (`core` + `revhub`), Marrow 1.2.5 (Weaver), FTC Dashboard 0.6.0, JUnit 4.13.2 local unit tests.

**Spec:** `docs/superpowers/specs/2026-11-09-perception-collect-design.md` (read it first).

## Global Constraints

- Java 8 only: no `var`, records, `List.of`, switch expressions, or diamond `<>` on anonymous classes.
- Coordinates: Pedro field frame everywhere (inches, 0 to 144, heading CCW+ radians, robot +X forward). Field geometry is written in the **manual frame** (x along the audience side from column A to F, y away from the audience, rows 1 to 6) and converted with `FieldFrame`.
- Limelight: `tx` + = right, `ty` + = up; pitch is **below** horizontal; staleness is in ms; `getControlHubTimeStamp()` is `System.currentTimeMillis()`-based ms.
- Class ids: 0 pollen, 1 red nectar, 2 blue nectar (load-bearing, never reorder).
- Every tunable lives in `PerceptionConfig` or `CollectConfig` (FTC Dashboard `@Config`); read at point of use.
- Never push this branch or open PRs without asking Tayyab.
- Unit tests: `./gradlew :TeamCode:testDebugUnitTest --tests '<fully.qualified.Class>'` from the repo root. Needs `local.properties` with `sdk.dir` (Task 1). A full run is the same command without `--tests`.
- Hardware glue (`CameraSubsystem`, `BallTrackingSubsystem`, `CollectCommand`, `PedroPathFollower`, `FieldDrawer`, OpModes) is compile-checked only: `./gradlew :TeamCode:compileDebugJavaWithJavac`.
- Commit after every task, ending each message with `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`.

Path shorthand used below (always expand it in commands):
- `MAIN` = `TeamCode/src/main/java/org/firstinspires/ftc/teamcode`
- `TEST` = `TeamCode/src/test/java/org/firstinspires/ftc/teamcode`

## File Structure

| File | Responsibility |
|------|----------------|
| `MAIN/perception/Angles.java` | angle wrap helper |
| `MAIN/perception/BallClass.java` | class id ↔ ball size |
| `MAIN/perception/PerceptionConfig.java` | all perception tunables |
| `MAIN/perception/CameraMount.java` | camera pose on the robot |
| `MAIN/perception/BallProjector.java` | tx/ty → field position, error model |
| `MAIN/perception/PoseSource.java`, `PoseHistory.java` | pose at frame capture time |
| `MAIN/perception/Detection.java`, `Sighting.java` | raw detection → field sighting with quality |
| `MAIN/perception/CameraView.java` | "should the camera see this point now?" |
| `MAIN/perception/MapBall.java`, `BallMap.java` | ball map + data association |
| `MAIN/perception/CameraMode.java`, `PipelineSpec.java`, `PipelineCycle.java`, `RawResult.java`, `CameraFrame.java`, `FrameGate.java` | frame gating (settle, staleness, pipeline) |
| `MAIN/perception/CameraSubsystem.java`, `BallTrackingSubsystem.java` | NextFTC hardware glue |
| `MAIN/collect/Alliance.java`, `CollectConfig.java`, `FieldRect.java`, `FieldFrame.java`, `FieldGeometry.java`, `BallSource.java`, `ZonePreset.java`, `CollectRequest.java`, `AllowedArea.java` | field model + strategy plug-in |
| `MAIN/collect/ObstacleProvider.java`, `CollectPlanner.java` | target choice + Weaver call |
| `MAIN/collect/SearchPlanner.java` | where to look when there are no targets |
| `MAIN/collect/ReplanPolicy.java` | when to re-plan |
| `MAIN/collect/IntakeStatus.java`, `PathFollower.java`, `CollectResult.java`, `CollectBrain.java` | collect state machine |
| `MAIN/collect/PedroPathFollower.java`, `CollectCommand.java` | NextFTC/Pedro glue |
| `MAIN/debug/FieldDrawer.java` | FTC Dashboard field overlay |
| `MAIN/opmodes/RobotHooks.java`, `CountingIntakeStatus.java`, `CollectTest.java` | test OpMode + integration hooks |

---

### Task 1: Branch, dependencies, test harness

**Files:**
- Modify: `build.dependencies.gradle`
- Modify: `TeamCode/build.gradle` (cherry-picked deps + dashboard + junit)
- Create: `local.properties` (gitignored, not committed)
- Test: `TEST/SmokeTest.java`

**Interfaces:**
- Produces: a branch where `./gradlew :TeamCode:testDebugUnitTest` runs JUnit 4 with Weaver, Pedro 3 and FTC Dashboard on the classpath.

- [ ] **Step 1: Create the branch with the spec, plan and dependency commit**

```bash
cd /Users/tayyabaamir/repositories/biobuzz
git switch -c feature/perception-collect design/perception-collect-spec
git cherry-pick a30d788
echo "sdk.dir=$HOME/Library/Android/sdk" > local.properties
```
Expected: cherry-pick succeeds; `TeamCode/build.gradle` now lists NextFTC 1.1.0, Pedro 3.0.2 and Marrow 1.2.5.

- [ ] **Step 2: Add the FTC Dashboard repository**

In `build.dependencies.gradle`, change the `repositories` block to:

```gradle
repositories {
    mavenCentral()
    google() // Needed for androidx
    maven { url = 'https://maven.brott.dev/' } // FTC Dashboard
}
```

- [ ] **Step 3: Add FTC Dashboard and JUnit to TeamCode**

Append inside the `dependencies { }` block of `TeamCode/build.gradle`, after the Marrow line:

```gradle
    // FTC Dashboard (live tuning + field drawing). It depends on SDK 10,
    // so the SDK group is excluded and our SDK 12 is used.
    implementation('com.acmerobotics.dashboard:dashboard:0.6.0') {
        exclude group: 'org.firstinspires.ftc'
    }
    // Laptop unit tests for the plain-Java perception/collect logic
    testImplementation 'junit:junit:4.13.2'
```

- [ ] **Step 4: Write the smoke test**

`TEST/SmokeTest.java`:

```java
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
```

- [ ] **Step 5: Run it**

Run: `./gradlew :TeamCode:testDebugUnitTest --tests 'org.firstinspires.ftc.teamcode.SmokeTest'`
Expected: `BUILD SUCCESSFUL`; `TeamCode/build/test-results/testDebugUnitTest/TEST-org.firstinspires.ftc.teamcode.SmokeTest.xml` shows `tests="1" failures="0" errors="0"`.

- [ ] **Step 6: Commit**

```bash
git add build.dependencies.gradle TeamCode/build.gradle TeamCode/src/test/java/org/firstinspires/ftc/teamcode/SmokeTest.java
git commit -m "Add FTC Dashboard and JUnit test harness" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 2: Camera geometry and ball projection

**Files:**
- Create: `MAIN/perception/Angles.java`, `MAIN/perception/BallClass.java`, `MAIN/perception/PerceptionConfig.java`, `MAIN/perception/CameraMount.java`, `MAIN/perception/BallProjector.java`
- Test: `TEST/perception/BallProjectorTest.java`

**Interfaces:**
- Produces:
  - `Angles.wrap(double rad) -> double` in (−π, π]
  - `BallClass {POLLEN, RED_NECTAR, BLUE_NECTAR}`, fields `classId`, `diameterIn`, `centerHeightIn()`, `static fromClassId(int) -> BallClass|null`
  - `CameraMount(forwardIn, leftIn, heightIn, pitchDownDeg, yawDeg, hfovDeg, vfovDeg)`, `fromConfig()`, `camX(Pose)`, `camY(Pose)`, `camYaw(Pose)`
  - `BallProjector.project(Pose robot, CameraMount m, double txDeg, double tyDeg, double ballCenterH) -> double[]{x, y, range}|null`
  - `BallProjector.errorPerDegree(double range, double camH, double ballH) -> double`
  - `PerceptionConfig` static fields (complete list below; later tasks use them)

- [ ] **Step 1: Write the failing test**

`TEST/perception/BallProjectorTest.java`:

```java
package org.firstinspires.ftc.teamcode.perception;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;

import org.junit.Test;

public class BallProjectorTest {
    private static final CameraMount PITCH30 = new CameraMount(0, 0, 12.4, 30, 0, 54.5, 42);
    private static final CameraMount LEVEL = new CameraMount(0, 0, 12.4, 0, 0, 54.5, 42);

    @Test
    public void opticalAxisHitsTheFloorAtHeightOverTanPitch() {
        double[] p = BallProjector.project(new Pose(0, 0, 0), PITCH30, 0, 0, 1.4);
        assertEquals(11.0 / Math.tan(Math.toRadians(30)), p[0], 1e-6);
        assertEquals(0, p[1], 1e-6);
        assertEquals(p[0], p[2], 1e-6);
    }

    @Test
    public void tySetsTheRange() {
        double ty = 30 - Math.toDegrees(Math.atan2(11, 40));
        double[] p = BallProjector.project(new Pose(0, 0, 0), PITCH30, 0, ty, 1.4);
        assertEquals(40, p[0], 1e-6);
        assertEquals(0, p[1], 1e-6);
    }

    @Test
    public void positiveTxIsToTheRight() {
        double tx = Math.toDegrees(Math.atan(10.0 / 40.0));
        double ty = Math.toDegrees(Math.atan(-11.0 / 40.0));
        double[] p = BallProjector.project(new Pose(0, 0, 0), LEVEL, tx, ty, 1.4);
        assertEquals(40, p[0], 1e-6);
        assertEquals(-10, p[1], 1e-6);
    }

    @Test
    public void cameraOffsetAndRobotHeadingAreApplied() {
        CameraMount m = new CameraMount(5, 0, 12.4, 30, 0, 54.5, 42);
        double[] p = BallProjector.project(new Pose(10, 20, Math.PI / 2), m, 0, 0, 1.4);
        assertEquals(10, p[0], 1e-6);
        assertEquals(25 + 11.0 / Math.tan(Math.toRadians(30)), p[1], 1e-6);
    }

    @Test
    public void rayAboveTheHorizonIsNull() {
        assertNull(BallProjector.project(new Pose(0, 0, 0), LEVEL, 0, 5, 1.4));
    }

    @Test
    public void errorPerDegreeMatchesTheFormulaAndGrowsWithRange() {
        assertEquals(1721.0 / 11.0 * Math.PI / 180.0, BallProjector.errorPerDegree(40, 12.4, 1.4), 1e-9);
        assertTrue(BallProjector.errorPerDegree(80, 12.4, 1.4) > 3 * BallProjector.errorPerDegree(40, 12.4, 1.4));
    }

    @Test
    public void classIdsAreLoadBearing() {
        assertEquals(BallClass.POLLEN, BallClass.fromClassId(0));
        assertEquals(BallClass.RED_NECTAR, BallClass.fromClassId(1));
        assertEquals(BallClass.BLUE_NECTAR, BallClass.fromClassId(2));
        assertNull(BallClass.fromClassId(3));
        assertEquals(1.4, BallClass.POLLEN.centerHeightIn(), 1e-9);
    }

    @Test
    public void wrapKeepsAnglesInRange() {
        assertEquals(Math.PI, Angles.wrap(Math.PI), 1e-12);
        assertEquals(-Math.PI / 2, Angles.wrap(3 * Math.PI / 2), 1e-12);
    }
}
```

- [ ] **Step 2: Run it to make sure it fails**

Run: `./gradlew :TeamCode:testDebugUnitTest --tests 'org.firstinspires.ftc.teamcode.perception.BallProjectorTest'`
Expected: FAIL at compile time with "cannot find symbol: class CameraMount".

- [ ] **Step 3: Implement**

`MAIN/perception/Angles.java`:

```java
package org.firstinspires.ftc.teamcode.perception;

/** Angle helpers. Radians, CCW positive. */
public final class Angles {
    private Angles() {
    }

    /** Wraps an angle into (-pi, pi]. */
    public static double wrap(double rad) {
        while (rad > Math.PI) {
            rad -= 2 * Math.PI;
        }
        while (rad <= -Math.PI) {
            rad += 2 * Math.PI;
        }
        return rad;
    }
}
```

`MAIN/perception/BallClass.java`:

```java
package org.firstinspires.ftc.teamcode.perception;

/** BIOBUZZ scoring elements. Class ids match the Limelight model's labels.txt order. */
public enum BallClass {
    POLLEN(0, 2.8),
    RED_NECTAR(1, 3.6),
    BLUE_NECTAR(2, 3.6);

    public final int classId;
    public final double diameterIn;

    BallClass(int classId, double diameterIn) {
        this.classId = classId;
        this.diameterIn = diameterIn;
    }

    /** Height of the ball centre above the tiles, in. */
    public double centerHeightIn() {
        return diameterIn / 2.0;
    }

    /** The class for a detector class id, or null for anything that is not a ball. */
    public static BallClass fromClassId(int id) {
        for (BallClass c : values()) {
            if (c.classId == id) {
                return c;
            }
        }
        return null;
    }
}
```

`MAIN/perception/PerceptionConfig.java`:

```java
package org.firstinspires.ftc.teamcode.perception;

import com.acmerobotics.dashboard.config.Config;

/** Every perception tunable. Editable live from FTC Dashboard; read at point of use. */
@Config("Perception")
public final class PerceptionConfig {
    private PerceptionConfig() {
    }

    // wiring
    public static String LIMELIGHT_NAME = "limelight";
    public static String TILT_SERVO_NAME = "camTilt";   // "" = fixed mount
    public static int GOAL_PIPELINE = 0;                 // AprilTag (fiducial) pipeline
    /** Ball pipelines, e.g. "1:NEURAL" or "1:COLOUR:POLLEN,2:COLOUR:RED_NECTAR,3:COLOUR:BLUE_NECTAR". */
    public static String BALL_PIPELINES = "1:NEURAL";
    public static double TILT_BALL_POS = 0.5;            // measure with the prelim bench
    public static double TILT_GOAL_POS = 0.5;

    // timing
    public static long MODE_SETTLE_MS = 250;             // servo move + pipeline switch
    public static long PIPELINE_SETTLE_MS = 100;         // pipeline switch only (colour cycling)
    public static long MAX_STALENESS_MS = 150;
    public static long POSE_HISTORY_MS = 500;

    // mount (from CAD at turret 0 deg; update when the mount is final)
    public static double CAM_FORWARD_IN = 0.0;
    public static double CAM_LEFT_IN = 0.0;
    public static double CAM_HEIGHT_IN = 12.4;
    public static double BALL_PITCH_DOWN_DEG = 25.0;     // set from the prelim bench
    public static double TURRET_BALL_YAW_DEG = 0.0;      // 180 later for the dual intake
    public static double HFOV_DEG = 54.5;
    public static double VFOV_DEG = 42.0;

    // sighting quality
    public static double MIN_CONFIDENCE = 0.25;          // junk floor only; real thresholds from capture data
    public static double GOOD_RANGE_IN = 48.0;
    public static double MAX_RANGE_IN = 120.0;
    public static double FAR_RANGE_FACTOR = 0.3;
    public static double EDGE_MARGIN_DEG = 4.0;
    public static double EDGE_FACTOR = 0.6;
    public static double ANGLE_SIGMA_DEG = 0.5;          // assumed angle noise -> position sigma
    public static double MIN_SIGMA_IN = 0.5;
    public static double TARGET_QUALITY = 0.5;
    public static double TARGET_SIGMA_IN = 2.0;
    public static double MAYBE_TWO_ASPECT = 1.6;

    // data association
    public static double GATE_SIGMAS = 3.0;
    public static double MIN_GATE_IN = 1.5;
    public static double TIGHT_GATE_DIAMETERS = 0.8;
    public static int CONFIRM_FRAMES = 2;
    public static long CANDIDATE_TIMEOUT_MS = 500;
    public static long FORGET_MS = 4000;
    public static int MISS_FRAMES_TO_REMOVE = 2;
    public static double VIEW_REMOVE_RANGE_IN = 60.0;

    // pickup bookkeeping
    public static double INTAKE_REACH_IN = 14.3;         // robot centre to intake mouth (CAD)
    public static double PICKUP_SEARCH_IN = 10.0;
}
```

`MAIN/perception/CameraMount.java`:

```java
package org.firstinspires.ftc.teamcode.perception;

import com.pedropathing.math.Pose;

/** Where the ball camera sits on the robot. Pitch is degrees BELOW horizontal; yaw is CCW from robot forward. */
public final class CameraMount {
    public final double forwardIn;
    public final double leftIn;
    public final double heightIn;
    public final double pitchDownDeg;
    public final double yawDeg;
    public final double hfovDeg;
    public final double vfovDeg;

    public CameraMount(double forwardIn, double leftIn, double heightIn, double pitchDownDeg,
                       double yawDeg, double hfovDeg, double vfovDeg) {
        this.forwardIn = forwardIn;
        this.leftIn = leftIn;
        this.heightIn = heightIn;
        this.pitchDownDeg = pitchDownDeg;
        this.yawDeg = yawDeg;
        this.hfovDeg = hfovDeg;
        this.vfovDeg = vfovDeg;
    }

    /** The ball-mode mount from PerceptionConfig. */
    public static CameraMount fromConfig() {
        return new CameraMount(PerceptionConfig.CAM_FORWARD_IN, PerceptionConfig.CAM_LEFT_IN,
                PerceptionConfig.CAM_HEIGHT_IN, PerceptionConfig.BALL_PITCH_DOWN_DEG,
                PerceptionConfig.TURRET_BALL_YAW_DEG, PerceptionConfig.HFOV_DEG, PerceptionConfig.VFOV_DEG);
    }

    public double camX(Pose robot) {
        double h = robot.heading();
        return robot.x() + forwardIn * Math.cos(h) - leftIn * Math.sin(h);
    }

    public double camY(Pose robot) {
        double h = robot.heading();
        return robot.y() + forwardIn * Math.sin(h) + leftIn * Math.cos(h);
    }

    public double camYaw(Pose robot) {
        return robot.heading() + Math.toRadians(yawDeg);
    }
}
```

`MAIN/perception/BallProjector.java`:

```java
package org.firstinspires.ftc.teamcode.perception;

import com.pedropathing.math.Pose;

/** Limelight angles -> field position. Port of Hive Vision BallMath.camRay/ballFieldPos. */
public final class BallProjector {
    private BallProjector() {
    }

    /**
     * Field position of a ball centre seen at (tx, ty).
     *
     * @return {x, y, horizontal range from the camera} in inches, or null if the ray never comes down to ballCenterH
     */
    public static double[] project(Pose robot, CameraMount m, double txDeg, double tyDeg, double ballCenterH) {
        double tx = Math.toRadians(txDeg);
        double ty = Math.toRadians(tyDeg);
        double p = Math.toRadians(m.pitchDownDeg);
        double txn = Math.tan(tx);
        double tyn = Math.tan(ty);
        double zc = 1.0 / Math.sqrt(1.0 + txn * txn + tyn * tyn);
        double xc = txn * zc;
        double yc = tyn * zc;
        double fwd = zc * Math.cos(p) + yc * Math.sin(p);
        double left = -xc;
        double up = -zc * Math.sin(p) + yc * Math.cos(p);
        double dz = ballCenterH - m.heightIn;
        if (up >= -1e-9 || dz >= 0) {
            return null;
        }
        double t = dz / up;
        double yaw = m.camYaw(robot);
        double wx = fwd * Math.cos(yaw) - left * Math.sin(yaw);
        double wy = fwd * Math.sin(yaw) + left * Math.cos(yaw);
        double cx = m.camX(robot);
        double cy = m.camY(robot);
        double bx = cx + t * wx;
        double by = cy + t * wy;
        return new double[]{bx, by, Math.hypot(bx - cx, by - cy)};
    }

    /** Floor position error (in) per degree of vertical angle error at a horizontal range. */
    public static double errorPerDegree(double range, double camH, double ballH) {
        double dz = camH - ballH;
        return (range * range + dz * dz) / dz * Math.PI / 180.0;
    }
}
```

- [ ] **Step 4: Run the test to make sure it passes**

Run: `./gradlew :TeamCode:testDebugUnitTest --tests 'org.firstinspires.ftc.teamcode.perception.BallProjectorTest'`
Expected: PASS (8 tests).

- [ ] **Step 5: Commit**

```bash
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/perception TeamCode/src/test/java/org/firstinspires/ftc/teamcode/perception
git commit -m "Add camera geometry and ball projection" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 3: Pose history

**Files:**
- Create: `MAIN/perception/PoseSource.java`, `MAIN/perception/PoseHistory.java`
- Test: `TEST/perception/PoseHistoryTest.java`

**Interfaces:**
- Consumes: `Angles.wrap`
- Produces: `interface PoseSource { Pose pose(); }`; `PoseHistory(long windowMs)`, `add(long tMs, Pose)`, `at(long tMs) -> Pose|null` (null if older than the history; newest pose if later than it)

- [ ] **Step 1: Write the failing test**

`TEST/perception/PoseHistoryTest.java`:

```java
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
```

- [ ] **Step 2: Run it to make sure it fails**

Run: `./gradlew :TeamCode:testDebugUnitTest --tests 'org.firstinspires.ftc.teamcode.perception.PoseHistoryTest'`
Expected: FAIL, "cannot find symbol: class PoseHistory".

- [ ] **Step 3: Implement**

`MAIN/perception/PoseSource.java`:

```java
package org.firstinspires.ftc.teamcode.perception;

import com.pedropathing.math.Pose;

/** Current robot pose in the Pedro field frame. In production: {@code follower::pose}. */
public interface PoseSource {
    Pose pose();
}
```

`MAIN/perception/PoseHistory.java`:

```java
package org.firstinspires.ftc.teamcode.perception;

import com.pedropathing.math.Pose;

import java.util.ArrayList;

/** Recent robot poses, so a camera frame can be paired with the pose at its capture time. */
public final class PoseHistory {
    private static final class Entry {
        final long t;
        final Pose pose;

        Entry(long t, Pose pose) {
            this.t = t;
            this.pose = pose;
        }
    }

    private final long windowMs;
    private final ArrayList<Entry> entries = new ArrayList<Entry>();

    public PoseHistory(long windowMs) {
        this.windowMs = windowMs;
    }

    /** Adds a sample. Samples must arrive in time order; out-of-order or null samples are ignored. */
    public synchronized void add(long tMs, Pose pose) {
        if (pose == null) {
            return;
        }
        if (!entries.isEmpty() && tMs <= entries.get(entries.size() - 1).t) {
            return;
        }
        entries.add(new Entry(tMs, pose));
        long cutoff = tMs - windowMs;
        while (entries.size() > 2 && entries.get(1).t <= cutoff) {
            entries.remove(0);
        }
    }

    /** Pose at tMs: interpolated inside the history, newest after it, null before it. */
    public synchronized Pose at(long tMs) {
        if (entries.isEmpty()) {
            return null;
        }
        Entry last = entries.get(entries.size() - 1);
        if (tMs >= last.t) {
            return last.pose;
        }
        if (tMs < entries.get(0).t) {
            return null;
        }
        for (int i = entries.size() - 1; i > 0; i--) {
            Entry a = entries.get(i - 1);
            Entry b = entries.get(i);
            if (tMs >= a.t) {
                double f = (double) (tMs - a.t) / (double) (b.t - a.t);
                return lerp(a.pose, b.pose, f);
            }
        }
        return entries.get(0).pose;
    }

    static Pose lerp(Pose a, Pose b, double f) {
        double dh = Angles.wrap(b.heading() - a.heading());
        return new Pose(a.x() + (b.x() - a.x()) * f,
                a.y() + (b.y() - a.y()) * f,
                Angles.wrap(a.heading() + dh * f));
    }
}
```

- [ ] **Step 4: Run the test to make sure it passes**

Run: `./gradlew :TeamCode:testDebugUnitTest --tests 'org.firstinspires.ftc.teamcode.perception.PoseHistoryTest'`
Expected: PASS (5 tests).

- [ ] **Step 5: Commit**

```bash
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/perception/PoseSource.java TeamCode/src/main/java/org/firstinspires/ftc/teamcode/perception/PoseHistory.java TeamCode/src/test/java/org/firstinspires/ftc/teamcode/perception/PoseHistoryTest.java
git commit -m "Add pose history for capture-time poses" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 4: Detections, sightings with quality, camera view

**Files:**
- Create: `MAIN/perception/Detection.java`, `MAIN/perception/Sighting.java`, `MAIN/perception/CameraView.java`
- Test: `TEST/perception/SightingTest.java`

**Interfaces:**
- Consumes: `BallProjector`, `BallClass`, `CameraMount`, `PerceptionConfig`, `Angles`
- Produces:
  - `Detection(int classId, double confidence, double txDeg, double tyDeg, double aspect)`, `withClassId(int)`, `static aspectFromCorners(List<List<Double>>) -> double`
  - `Sighting(BallClass, double x, double y, double range, double sigma, double quality, boolean maybeTwo)` (public final fields of the same names, `ballClass` for the class)
  - `Sighting.from(Detection, Pose robot, CameraMount) -> Sighting|null`
  - `Sighting.quality(double conf, double range, double txDeg, double tyDeg, CameraMount) -> double`
  - `CameraView.canSee(Pose robot, CameraMount, double px, double py, double ballH, double maxRange) -> boolean`

- [ ] **Step 1: Write the failing test**

`TEST/perception/SightingTest.java`:

```java
package org.firstinspires.ftc.teamcode.perception;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;

import org.junit.Test;

import java.util.Arrays;
import java.util.List;

public class SightingTest {
    private static final CameraMount M = new CameraMount(0, 0, 12.4, 25, 0, 54.5, 42);
    private static final Pose ORIGIN = new Pose(0, 0, 0);

    private static double tyForRange(double range) {
        return 25 - Math.toDegrees(Math.atan2(11, range));
    }

    @Test
    public void lowConfidenceAndNonBallsAreDropped() {
        assertNull(Sighting.from(new Detection(0, 0.1, 0, tyForRange(40), 1), ORIGIN, M));
        assertNull(Sighting.from(new Detection(7, 0.9, 0, tyForRange(40), 1), ORIGIN, M));
    }

    @Test
    public void closeCentredSightingKeepsItsConfidenceAsQuality() {
        Sighting s = Sighting.from(new Detection(0, 0.9, 0, tyForRange(40), 1), ORIGIN, M);
        assertEquals(40, s.x, 1e-6);
        assertEquals(0.9, s.quality, 1e-9);
        assertEquals(BallProjector.errorPerDegree(40, 12.4, 1.4) * 0.5, s.sigma, 1e-9);
        assertFalse(s.maybeTwo);
    }

    @Test
    public void farAndEdgeSightingsLoseQuality() {
        assertEquals(0.9 * 0.3, Sighting.quality(0.9, 200, 0, 0, M), 1e-9);
        assertEquals(0.9 * 0.65, Sighting.quality(0.9, 84, 0, 0, M), 1e-9);
        assertEquals(0.9 * 0.6, Sighting.quality(0.9, 30, 26, 0, M), 1e-9);
    }

    @Test
    public void wideBoxesAreFlaggedAsMaybeTwo() {
        Sighting s = Sighting.from(new Detection(0, 0.9, 0, tyForRange(40), 2.0), ORIGIN, M);
        assertTrue(s.maybeTwo);
    }

    @Test
    public void aspectComesFromBoxCorners() {
        List<List<Double>> corners = Arrays.asList(
                Arrays.asList(10.0, 10.0), Arrays.asList(30.0, 10.0),
                Arrays.asList(30.0, 20.0), Arrays.asList(10.0, 20.0));
        assertEquals(2.0, Detection.aspectFromCorners(corners), 1e-9);
        assertEquals(1.0, Detection.aspectFromCorners(null), 1e-9);
    }

    @Test
    public void cameraViewMatchesTheFieldOfView() {
        assertTrue(CameraView.canSee(ORIGIN, M, 40, 0, 1.4, 60));
        assertFalse(CameraView.canSee(new Pose(0, 0, Math.PI), M, 40, 0, 1.4, 60));
        assertFalse(CameraView.canSee(ORIGIN, M, 10, 0, 1.4, 60));   // below the bottom edge
        assertFalse(CameraView.canSee(ORIGIN, M, 40, 30, 1.4, 60));  // left of the frame
        assertFalse(CameraView.canSee(ORIGIN, M, 70, 0, 1.4, 60));   // beyond maxRange
    }
}
```

Quality arithmetic for the 84 in case: f = (84 − 48) / (120 − 48) = 0.5, factor = 1 − 0.5 × 0.7 = 0.65.

- [ ] **Step 2: Run it to make sure it fails**

Run: `./gradlew :TeamCode:testDebugUnitTest --tests 'org.firstinspires.ftc.teamcode.perception.SightingTest'`
Expected: FAIL, "cannot find symbol: class Detection".

- [ ] **Step 3: Implement**

`MAIN/perception/Detection.java`:

```java
package org.firstinspires.ftc.teamcode.perception;

import java.util.List;

/** One detector (or colour) result in camera angles. aspect = box width / height (1 if unknown). */
public final class Detection {
    public final int classId;
    public final double confidence;
    public final double txDeg;
    public final double tyDeg;
    public final double aspect;

    public Detection(int classId, double confidence, double txDeg, double tyDeg, double aspect) {
        this.classId = classId;
        this.confidence = confidence;
        this.txDeg = txDeg;
        this.tyDeg = tyDeg;
        this.aspect = aspect;
    }

    public Detection withClassId(int id) {
        return new Detection(id, confidence, txDeg, tyDeg, aspect);
    }

    /** Width/height of the box spanned by Limelight target corners ([[x, y], ...] in pixels). */
    public static double aspectFromCorners(List<List<Double>> corners) {
        if (corners == null || corners.isEmpty()) {
            return 1.0;
        }
        double minX = Double.MAX_VALUE;
        double maxX = -Double.MAX_VALUE;
        double minY = Double.MAX_VALUE;
        double maxY = -Double.MAX_VALUE;
        for (List<Double> c : corners) {
            if (c == null || c.size() < 2) {
                continue;
            }
            minX = Math.min(minX, c.get(0));
            maxX = Math.max(maxX, c.get(0));
            minY = Math.min(minY, c.get(1));
            maxY = Math.max(maxY, c.get(1));
        }
        double h = maxY - minY;
        double w = maxX - minX;
        return h > 1e-9 && w > 0 ? w / h : 1.0;
    }
}
```

`MAIN/perception/Sighting.java`:

```java
package org.firstinspires.ftc.teamcode.perception;

import com.pedropathing.math.Pose;

/** A ball seen in one frame, in the field frame, with its position uncertainty and quality. */
public final class Sighting {
    public final BallClass ballClass;
    public final double x;
    public final double y;
    public final double range;
    public final double sigma;
    public final double quality;
    public final boolean maybeTwo;

    public Sighting(BallClass ballClass, double x, double y, double range, double sigma,
                    double quality, boolean maybeTwo) {
        this.ballClass = ballClass;
        this.x = x;
        this.y = y;
        this.range = range;
        this.sigma = sigma;
        this.quality = quality;
        this.maybeTwo = maybeTwo;
    }

    /** Projects a detection, or returns null for non-balls, junk confidence or rays that miss the floor. */
    public static Sighting from(Detection d, Pose robot, CameraMount m) {
        BallClass c = BallClass.fromClassId(d.classId);
        if (c == null || d.confidence < PerceptionConfig.MIN_CONFIDENCE) {
            return null;
        }
        double[] p = BallProjector.project(robot, m, d.txDeg, d.tyDeg, c.centerHeightIn());
        if (p == null) {
            return null;
        }
        double sigma = Math.max(PerceptionConfig.MIN_SIGMA_IN,
                BallProjector.errorPerDegree(p[2], m.heightIn, c.centerHeightIn()) * PerceptionConfig.ANGLE_SIGMA_DEG);
        return new Sighting(c, p[0], p[1], p[2], sigma,
                quality(d.confidence, p[2], d.txDeg, d.tyDeg, m),
                d.aspect > PerceptionConfig.MAYBE_TWO_ASPECT);
    }

    /** Confidence x range factor x frame-edge factor. */
    public static double quality(double conf, double range, double txDeg, double tyDeg, CameraMount m) {
        double good = PerceptionConfig.GOOD_RANGE_IN;
        double max = PerceptionConfig.MAX_RANGE_IN;
        double far = PerceptionConfig.FAR_RANGE_FACTOR;
        double rangeFactor;
        if (range <= good) {
            rangeFactor = 1.0;
        } else if (range >= max) {
            rangeFactor = far;
        } else {
            rangeFactor = 1.0 - (range - good) / (max - good) * (1.0 - far);
        }
        double margin = PerceptionConfig.EDGE_MARGIN_DEG;
        boolean edge = Math.abs(txDeg) > m.hfovDeg / 2 - margin || Math.abs(tyDeg) > m.vfovDeg / 2 - margin;
        return conf * rangeFactor * (edge ? PerceptionConfig.EDGE_FACTOR : 1.0);
    }
}
```

`MAIN/perception/CameraView.java`:

```java
package org.firstinspires.ftc.teamcode.perception;

import com.pedropathing.math.Pose;

/** Whether a floor point is inside the camera's usable view (FOV minus the edge margin, within maxRange). */
public final class CameraView {
    private CameraView() {
    }

    public static boolean canSee(Pose robot, CameraMount m, double px, double py, double ballH, double maxRange) {
        double cx = m.camX(robot);
        double cy = m.camY(robot);
        double dx = px - cx;
        double dy = py - cy;
        double r = Math.hypot(dx, dy);
        if (r > maxRange || r < 1e-6) {
            return false;
        }
        double bearingDeg = Math.toDegrees(Angles.wrap(Math.atan2(dy, dx) - m.camYaw(robot)));
        double txDeg = -bearingDeg;
        double depressionDeg = Math.toDegrees(Math.atan2(m.heightIn - ballH, r));
        double tyDeg = m.pitchDownDeg - depressionDeg;
        double margin = PerceptionConfig.EDGE_MARGIN_DEG;
        return Math.abs(txDeg) <= m.hfovDeg / 2 - margin && Math.abs(tyDeg) <= m.vfovDeg / 2 - margin;
    }
}
```

- [ ] **Step 4: Run the test to make sure it passes**

Run: `./gradlew :TeamCode:testDebugUnitTest --tests 'org.firstinspires.ftc.teamcode.perception.SightingTest'`
Expected: PASS (6 tests).

- [ ] **Step 5: Commit**

```bash
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/perception TeamCode/src/test/java/org/firstinspires/ftc/teamcode/perception/SightingTest.java
git commit -m "Add sightings with quality and camera view check" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 5: Ball map with one-to-one data association

**Files:**
- Create: `MAIN/perception/MapBall.java`, `MAIN/perception/BallMap.java`
- Test: `TEST/perception/BallMapTest.java`

**Interfaces:**
- Consumes: `Sighting`, `CameraView`, `CameraMount`, `PerceptionConfig`
- Produces:
  - `MapBall`: `id()`, `ballClass()`, `x()`, `y()`, `sigma()`, `quality()`, `seenCount()`, `lastSeenMs()`, `maybeTwo()`, `isConfirmed()`, `isTarget()`, `isHint()`, `distanceTo(double, double)`
  - `BallMap`: `update(long nowMs, Pose robot|null, CameraMount|null, List<Sighting>)`, `prune(long nowMs)`, `removePickup(Pose robot) -> MapBall|null`, `all()`, `targets()`, `hints()`, `contains(int id)`, `size()`, `clear()`

- [ ] **Step 1: Write the failing test**

`TEST/perception/BallMapTest.java`:

```java
package org.firstinspires.ftc.teamcode.perception;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class BallMapTest {
    private static final CameraMount M = new CameraMount(0, 0, 12.4, 25, 0, 54.5, 42);

    static Sighting s(double x, double y, double sigma, double quality) {
        return new Sighting(BallClass.POLLEN, x, y, 40, sigma, quality, false);
    }

    static List<Sighting> list(Sighting... ss) {
        return new ArrayList<Sighting>(Arrays.asList(ss));
    }

    @Test
    public void twoSightingsInOneFrameAreNeverMerged() {
        BallMap map = new BallMap();
        map.update(0, null, null, list(s(40, 0, 1.0, 0.9), s(40, 1.0, 1.0, 0.9)));
        assertEquals(2, map.size());
    }

    @Test
    public void touchingPollenStaySeparateAcrossFrames() {
        BallMap map = new BallMap();
        map.update(0, null, null, list(s(40, 0, 1.0, 0.9), s(40, 2.8, 1.0, 0.9)));
        map.update(50, null, null, list(s(40, 0.3, 1.0, 0.9), s(40, 3.1, 1.0, 0.9)));
        map.update(100, null, null, list(s(40, -0.3, 1.0, 0.9), s(40, 2.5, 1.0, 0.9)));
        assertEquals(2, map.size());
        assertTrue(map.all().get(0).isConfirmed());
        assertTrue(map.all().get(1).isConfirmed());
    }

    @Test
    public void newNeighbourOfAConfirmedBallBecomesItsOwnBall() {
        BallMap map = new BallMap();
        map.update(0, null, null, list(s(40, 0, 1.0, 0.9)));
        map.update(50, null, null, list(s(40, 0, 1.0, 0.9)));
        map.update(100, null, null, list(s(40, 0.2, 1.0, 0.9), s(40, 2.6, 1.0, 0.9)));
        assertEquals(2, map.size());
    }

    @Test
    public void farHintSplitsWhenSeenUpClose() {
        BallMap map = new BallMap();
        map.update(0, null, null, list(s(100, 0, 6.0, 0.25)));
        map.update(50, null, null, list(s(100, 0, 6.0, 0.25)));
        assertEquals(1, map.hints().size());
        map.update(100, null, null, list(s(100, -1.4, 1.0, 0.9), s(100, 1.4, 1.0, 0.9)));
        assertEquals(2, map.size());
    }

    @Test
    public void aBallNeedsTwoFramesToBeConfirmed() {
        BallMap map = new BallMap();
        map.update(0, null, null, list(s(40, 0, 1.0, 0.9)));
        assertFalse(map.all().get(0).isConfirmed());
        assertTrue(map.targets().isEmpty());
        map.update(50, null, null, list(s(40, 0.1, 1.0, 0.9)));
        assertTrue(map.all().get(0).isConfirmed());
        assertEquals(1, map.targets().size());
    }

    @Test
    public void unconfirmedCandidatesExpire() {
        BallMap map = new BallMap();
        map.update(0, null, null, list(s(40, 0, 1.0, 0.9)));
        map.prune(600);
        assertEquals(0, map.size());
    }

    @Test
    public void confirmedBallsAreForgottenAfterFourSeconds() {
        BallMap map = new BallMap();
        map.update(0, null, null, list(s(40, 0, 1.0, 0.9)));
        map.update(50, null, null, list(s(40, 0, 1.0, 0.9)));
        map.prune(4000);
        assertEquals(1, map.size());
        map.prune(4100);
        assertEquals(0, map.size());
    }

    @Test
    public void ballInViewButNotSeenIsRemovedAfterTwoFrames() {
        BallMap map = new BallMap();
        Pose robot = new Pose(0, 0, 0);
        map.update(0, robot, M, list(s(40, 0, 1.0, 0.9)));
        map.update(50, robot, M, list(s(40, 0, 1.0, 0.9)));
        map.update(100, robot, M, Collections.<Sighting>emptyList());
        assertEquals(1, map.size());
        map.update(150, robot, M, Collections.<Sighting>emptyList());
        assertEquals(0, map.size());
    }

    @Test
    public void ballOutOfViewIsKept() {
        BallMap map = new BallMap();
        map.update(0, new Pose(0, 0, 0), M, list(s(40, 0, 1.0, 0.9)));
        map.update(50, new Pose(0, 0, 0), M, list(s(40, 0, 1.0, 0.9)));
        Pose facingAway = new Pose(0, 0, Math.PI);
        map.update(100, facingAway, M, Collections.<Sighting>emptyList());
        map.update(150, facingAway, M, Collections.<Sighting>emptyList());
        assertEquals(1, map.size());
    }

    @Test
    public void pickupRemovesExactlyOneBall() {
        BallMap map = new BallMap();
        map.update(0, null, null, list(s(15, 0, 1.0, 0.9), s(15, 2.8, 1.0, 0.9)));
        map.update(50, null, null, list(s(15, 0, 1.0, 0.9), s(15, 2.8, 1.0, 0.9)));
        MapBall taken = map.removePickup(new Pose(0, 0, 0));
        assertNotNull(taken);
        assertEquals(15, taken.x(), 0.2);
        assertEquals(0, taken.y(), 0.2);
        assertEquals(1, map.size());
        assertEquals(2.8, map.all().get(0).y(), 0.2);
    }

    @Test
    public void maybeTwoBallSurvivesTheFirstPickup() {
        BallMap map = new BallMap();
        Sighting two = new Sighting(BallClass.POLLEN, 15, 0, 15, 1.0, 0.9, true);
        map.update(0, null, null, list(two));
        map.update(50, null, null, list(two));
        assertNotNull(map.removePickup(new Pose(0, 0, 0)));
        assertEquals(1, map.size());
        assertFalse(map.all().get(0).maybeTwo());
        assertNotNull(map.removePickup(new Pose(0, 0, 0)));
        assertEquals(0, map.size());
    }

    @Test
    public void differentClassesNeverMatch() {
        BallMap map = new BallMap();
        map.update(0, null, null, list(s(40, 0, 1.0, 0.9)));
        map.update(50, null, null, list(new Sighting(BallClass.RED_NECTAR, 40, 0, 40, 1.0, 0.9, false)));
        assertEquals(2, map.size());
    }
}
```

- [ ] **Step 2: Run it to make sure it fails**

Run: `./gradlew :TeamCode:testDebugUnitTest --tests 'org.firstinspires.ftc.teamcode.perception.BallMapTest'`
Expected: FAIL, "cannot find symbol: class BallMap".

- [ ] **Step 3: Implement**

`MAIN/perception/MapBall.java`:

```java
package org.firstinspires.ftc.teamcode.perception;

/** One ball in the field-frame map. Mutated only by BallMap. */
public final class MapBall {
    private final int id;
    private final BallClass ballClass;
    double x;
    double y;
    double sigma;
    double quality;
    int seenCount;
    long lastSeenMs;
    int missFrames;
    boolean maybeTwo;

    MapBall(int id, Sighting s, long nowMs) {
        this.id = id;
        this.ballClass = s.ballClass;
        this.x = s.x;
        this.y = s.y;
        this.sigma = s.sigma;
        this.quality = s.quality;
        this.seenCount = 1;
        this.lastSeenMs = nowMs;
        this.maybeTwo = s.maybeTwo;
    }

    public int id() { return id; }
    public BallClass ballClass() { return ballClass; }
    public double x() { return x; }
    public double y() { return y; }
    public double sigma() { return sigma; }
    public double quality() { return quality; }
    public int seenCount() { return seenCount; }
    public long lastSeenMs() { return lastSeenMs; }
    public boolean maybeTwo() { return maybeTwo; }

    public boolean isConfirmed() {
        return seenCount >= PerceptionConfig.CONFIRM_FRAMES;
    }

    /** Good enough to drive to. */
    public boolean isTarget() {
        return isConfirmed() && quality >= PerceptionConfig.TARGET_QUALITY && isPrecise();
    }

    /** Seen, but too far/uncertain to drive to: worth a closer look. */
    public boolean isHint() {
        return isConfirmed() && !isTarget();
    }

    boolean isPrecise() {
        return sigma <= PerceptionConfig.TARGET_SIGMA_IN;
    }

    public double distanceTo(double px, double py) {
        return Math.hypot(x - px, y - py);
    }
}
```

`MAIN/perception/BallMap.java`:

```java
package org.firstinspires.ftc.teamcode.perception;

import com.pedropathing.math.Pose;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Iterator;
import java.util.List;

/**
 * Field-frame ball map. Data association is one-to-one per frame: each sighting updates at
 * most one ball and each ball takes at most one sighting, so two balls are never merged.
 */
public final class BallMap {
    private final ArrayList<MapBall> balls = new ArrayList<MapBall>();
    private int nextId = 1;

    /**
     * Folds one frame's sightings into the map. robot/mount may be null (tests); then the
     * "in view but not seen" removal is skipped.
     */
    public synchronized void update(long nowMs, Pose robot, CameraMount mount, List<Sighting> sightings) {
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
                if (CameraView.canSee(robot, mount, b.x, b.y, b.ballClass().centerHeightIn(),
                        PerceptionConfig.VIEW_REMOVE_RANGE_IN)) {
                    b.missFrames++;
                    if (b.missFrames >= PerceptionConfig.MISS_FRAMES_TO_REMOVE) {
                        gone.add(b);
                    }
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

    /** Tight gate (below one ball diameter) when both are precise; statistical gate otherwise. */
    static double gate(Sighting s, MapBall b) {
        double statistical = Math.max(PerceptionConfig.MIN_GATE_IN,
                PerceptionConfig.GATE_SIGMAS * Math.max(s.sigma, b.sigma));
        boolean precise = s.sigma <= PerceptionConfig.TARGET_SIGMA_IN && b.isPrecise();
        if (!precise) {
            return statistical;
        }
        return Math.min(statistical, PerceptionConfig.TIGHT_GATE_DIAMETERS * s.ballClass.diameterIn);
    }

    private static void fuse(MapBall b, Sighting s, long nowMs) {
        boolean sharper = s.sigma <= b.sigma;
        double wb = 1.0 / (b.sigma * b.sigma);
        double ws = 1.0 / (s.sigma * s.sigma);
        b.x = (b.x * wb + s.x * ws) / (wb + ws);
        b.y = (b.y * wb + s.y * ws) / (wb + ws);
        b.sigma = Math.max(PerceptionConfig.MIN_SIGMA_IN, Math.sqrt(1.0 / (wb + ws)));
        b.quality = Math.max(b.quality, s.quality);
        if (sharper) {
            b.maybeTwo = s.maybeTwo;
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
```

- [ ] **Step 4: Run the test to make sure it passes**

Run: `./gradlew :TeamCode:testDebugUnitTest --tests 'org.firstinspires.ftc.teamcode.perception.BallMapTest'`
Expected: PASS (12 tests).

- [ ] **Step 5: Commit**

```bash
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/perception/MapBall.java TeamCode/src/main/java/org/firstinspires/ftc/teamcode/perception/BallMap.java TeamCode/src/test/java/org/firstinspires/ftc/teamcode/perception/BallMapTest.java
git commit -m "Add ball map with one-to-one data association" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 6: Frame gating and pipeline cycling

**Files:**
- Create: `MAIN/perception/CameraMode.java`, `MAIN/perception/PipelineSpec.java`, `MAIN/perception/PipelineCycle.java`, `MAIN/perception/RawResult.java`, `MAIN/perception/CameraFrame.java`, `MAIN/perception/FrameGate.java`
- Test: `TEST/perception/FrameGateTest.java`

**Interfaces:**
- Consumes: `Detection`, `PoseHistory`, `BallClass`, `PerceptionConfig`
- Produces:
  - `enum CameraMode { BALL, GOAL }`
  - `PipelineSpec.neural(int)`, `PipelineSpec.colour(int, BallClass)`, fields `index`, `type` (`PipelineSpec.Type.NEURAL|COLOUR`), `colourClass`
  - `PipelineCycle.parse(String)`, `current()`, `advance()`, `size()`
  - `RawResult(long chTimestampMs, long stalenessMs, int pipelineIndex, double captureLatencyMs, double targetingLatencyMs, List<Detection> detections, List<Detection> colourBlobs)`
  - `CameraFrame` fields `id`, `mode`, `pipeline`, `captureTimeMs`, `robotPose`, `balls`
  - `FrameGate.expect(int pipelineIndex, long nowMs, long settleMs)`, `accept(RawResult, long nowMs, CameraMode, PipelineSpec|null, PoseHistory, boolean turretOk) -> CameraFrame|null`, `lastReject() -> FrameGate.Reject`

- [ ] **Step 1: Write the failing test**

`TEST/perception/FrameGateTest.java`:

```java
package org.firstinspires.ftc.teamcode.perception;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;

import com.pedropathing.math.Pose;

import org.junit.Before;
import org.junit.Test;

import java.util.Collections;
import java.util.List;

public class FrameGateTest {
    private PoseHistory poses;
    private FrameGate gate;
    private static final PipelineSpec NEURAL = PipelineSpec.neural(1);

    @Before
    public void setUp() {
        poses = new PoseHistory(5000);
        poses.add(0, new Pose(0, 0, 0));
        poses.add(1000, new Pose(100, 0, 0));
        gate = new FrameGate();
        gate.expect(1, 0, 0);
    }

    private static RawResult raw(long ts, int pipeline, long stale, List<Detection> det, List<Detection> col) {
        return new RawResult(ts, stale, pipeline, 20, 30, det, col);
    }

    private static List<Detection> one(int classId) {
        return Collections.singletonList(new Detection(classId, 0.9, 1, 2, 1));
    }

    @Test
    public void acceptsAndStampsThePoseAtCaptureTime() {
        CameraFrame f = gate.accept(raw(500, 1, 10, one(0), Collections.<Detection>emptyList()),
                520, CameraMode.BALL, NEURAL, poses, true);
        assertNotNull(f);
        assertEquals(1, f.id);
        assertEquals(450, f.captureTimeMs);
        assertEquals(45, f.robotPose.x(), 1e-9);
        assertEquals(1, f.balls.size());
        assertEquals(FrameGate.Reject.NONE, gate.lastReject());
    }

    @Test
    public void sameTimestampIsNotNew() {
        RawResult r = raw(500, 1, 10, one(0), Collections.<Detection>emptyList());
        assertNotNull(gate.accept(r, 520, CameraMode.BALL, NEURAL, poses, true));
        assertNull(gate.accept(r, 530, CameraMode.BALL, NEURAL, poses, true));
        assertEquals(FrameGate.Reject.NOT_NEW, gate.lastReject());
    }

    @Test
    public void wrongPipelineIsRejected() {
        assertNull(gate.accept(raw(500, 0, 10, one(0), Collections.<Detection>emptyList()),
                520, CameraMode.BALL, NEURAL, poses, true));
        assertEquals(FrameGate.Reject.WRONG_PIPELINE, gate.lastReject());
    }

    @Test
    public void framesWaitForTheSettleTime() {
        gate.expect(1, 400, 250);
        assertNull(gate.accept(raw(500, 1, 10, one(0), Collections.<Detection>emptyList()),
                500, CameraMode.BALL, NEURAL, poses, true));
        assertEquals(FrameGate.Reject.SETTLING, gate.lastReject());
        assertNotNull(gate.accept(raw(700, 1, 10, one(0), Collections.<Detection>emptyList()),
                700, CameraMode.BALL, NEURAL, poses, true));
    }

    @Test
    public void staleResultsAreRejected() {
        assertNull(gate.accept(raw(500, 1, 200, one(0), Collections.<Detection>emptyList()),
                520, CameraMode.BALL, NEURAL, poses, true));
        assertEquals(FrameGate.Reject.STALE, gate.lastReject());
    }

    @Test
    public void turretOffAngleRejectsBallFramesOnly() {
        assertNull(gate.accept(raw(500, 1, 10, one(0), Collections.<Detection>emptyList()),
                520, CameraMode.BALL, NEURAL, poses, false));
        assertEquals(FrameGate.Reject.TURRET, gate.lastReject());
        gate.expect(0, 0, 0);
        CameraFrame goal = gate.accept(raw(600, 0, 10, one(0), Collections.<Detection>emptyList()),
                620, CameraMode.GOAL, null, poses, false);
        assertNotNull(goal);
        assertEquals(0, goal.balls.size());
    }

    @Test
    public void colourPipelineAssignsItsClass() {
        gate.expect(2, 0, 0);
        CameraFrame f = gate.accept(raw(500, 2, 10, Collections.<Detection>emptyList(), one(-1)),
                520, CameraMode.BALL, PipelineSpec.colour(2, BallClass.RED_NECTAR), poses, true);
        assertEquals(1, f.balls.get(0).classId);
    }

    @Test
    public void emptyFramesStillCount() {
        CameraFrame f = gate.accept(raw(500, 1, 10, Collections.<Detection>emptyList(),
                Collections.<Detection>emptyList()), 520, CameraMode.BALL, NEURAL, poses, true);
        assertNotNull(f);
        assertEquals(0, f.balls.size());
    }

    @Test
    public void pipelineCycleParsesAndRotates() {
        assertEquals(1, PipelineCycle.parse("1:NEURAL").size());
        PipelineCycle c = PipelineCycle.parse("1:COLOUR:POLLEN, 2:COLOUR:RED_NECTAR");
        assertEquals(1, c.current().index);
        assertEquals(BallClass.POLLEN, c.current().colourClass);
        assertEquals(2, c.advance().index);
        assertEquals(1, c.advance().index);
    }

    @Test(expected = IllegalArgumentException.class)
    public void badPipelineTextIsRejected() {
        PipelineCycle.parse("1:FISH");
    }
}
```

- [ ] **Step 2: Run it to make sure it fails**

Run: `./gradlew :TeamCode:testDebugUnitTest --tests 'org.firstinspires.ftc.teamcode.perception.FrameGateTest'`
Expected: FAIL, "cannot find symbol: class FrameGate".

- [ ] **Step 3: Implement**

`MAIN/perception/CameraMode.java`:

```java
package org.firstinspires.ftc.teamcode.perception;

/** BALL = tilted down, ball pipeline(s). GOAL = tilted up, AprilTag pipeline. */
public enum CameraMode {
    BALL,
    GOAL
}
```

`MAIN/perception/PipelineSpec.java`:

```java
package org.firstinspires.ftc.teamcode.perception;

/** One Limelight ball pipeline. A NEURAL pipeline reports all classes; a COLOUR pipeline tracks one class. */
public final class PipelineSpec {
    public enum Type { NEURAL, COLOUR }

    public final int index;
    public final Type type;
    public final BallClass colourClass;

    private PipelineSpec(int index, Type type, BallClass colourClass) {
        this.index = index;
        this.type = type;
        this.colourClass = colourClass;
    }

    public static PipelineSpec neural(int index) {
        return new PipelineSpec(index, Type.NEURAL, null);
    }

    public static PipelineSpec colour(int index, BallClass c) {
        return new PipelineSpec(index, Type.COLOUR, c);
    }
}
```

`MAIN/perception/PipelineCycle.java`:

```java
package org.firstinspires.ftc.teamcode.perception;

import java.util.ArrayList;
import java.util.List;

/** The ball pipelines, visited round-robin when there is more than one. */
public final class PipelineCycle {
    private final List<PipelineSpec> specs;
    private int current = 0;

    public PipelineCycle(List<PipelineSpec> specs) {
        if (specs == null || specs.isEmpty()) {
            throw new IllegalArgumentException("At least one ball pipeline is required");
        }
        this.specs = new ArrayList<PipelineSpec>(specs);
    }

    /** Parses "1:NEURAL" or "1:COLOUR:POLLEN,2:COLOUR:RED_NECTAR,...". */
    public static PipelineCycle parse(String text) {
        ArrayList<PipelineSpec> out = new ArrayList<PipelineSpec>();
        for (String part : text.split(",")) {
            String[] f = part.trim().split(":");
            if (f.length == 2 && f[1].trim().equals("NEURAL")) {
                out.add(PipelineSpec.neural(Integer.parseInt(f[0].trim())));
            } else if (f.length == 3 && f[1].trim().equals("COLOUR")) {
                out.add(PipelineSpec.colour(Integer.parseInt(f[0].trim()), BallClass.valueOf(f[2].trim())));
            } else {
                throw new IllegalArgumentException("Bad ball pipeline entry: " + part);
            }
        }
        return new PipelineCycle(out);
    }

    public PipelineSpec current() {
        return specs.get(current);
    }

    public PipelineSpec advance() {
        current = (current + 1) % specs.size();
        return current();
    }

    public int size() {
        return specs.size();
    }
}
```

`MAIN/perception/RawResult.java`:

```java
package org.firstinspires.ftc.teamcode.perception;

import java.util.List;

/** The parts of an LLResult we use, copied out so gating can be unit-tested without the SDK. */
public final class RawResult {
    public final long chTimestampMs;
    public final long stalenessMs;
    public final int pipelineIndex;
    public final double captureLatencyMs;
    public final double targetingLatencyMs;
    public final List<Detection> detections;
    public final List<Detection> colourBlobs;

    public RawResult(long chTimestampMs, long stalenessMs, int pipelineIndex, double captureLatencyMs,
                     double targetingLatencyMs, List<Detection> detections, List<Detection> colourBlobs) {
        this.chTimestampMs = chTimestampMs;
        this.stalenessMs = stalenessMs;
        this.pipelineIndex = pipelineIndex;
        this.captureLatencyMs = captureLatencyMs;
        this.targetingLatencyMs = targetingLatencyMs;
        this.detections = detections;
        this.colourBlobs = colourBlobs;
    }
}
```

`MAIN/perception/CameraFrame.java`:

```java
package org.firstinspires.ftc.teamcode.perception;

import com.pedropathing.math.Pose;

import java.util.List;

/** A gated camera frame stamped with the robot pose at capture time. balls is empty in GOAL mode. */
public final class CameraFrame {
    public final long id;
    public final CameraMode mode;
    public final PipelineSpec pipeline;
    public final long captureTimeMs;
    public final Pose robotPose;
    public final List<Detection> balls;

    public CameraFrame(long id, CameraMode mode, PipelineSpec pipeline, long captureTimeMs,
                       Pose robotPose, List<Detection> balls) {
        this.id = id;
        this.mode = mode;
        this.pipeline = pipeline;
        this.captureTimeMs = captureTimeMs;
        this.robotPose = robotPose;
        this.balls = balls;
    }
}
```

`MAIN/perception/FrameGate.java`:

```java
package org.firstinspires.ftc.teamcode.perception;

import com.pedropathing.math.Pose;

import java.util.ArrayList;
import java.util.Collections;

/**
 * Decides whether a Limelight result becomes a frame: new, from the expected pipeline, fresh,
 * past the settle time, turret in place, and with a pose at its capture time.
 */
public final class FrameGate {
    public enum Reject { NONE, NO_RESULT, NOT_NEW, WRONG_PIPELINE, STALE, SETTLING, TURRET, NO_POSE }

    private long lastTimestampMs = Long.MIN_VALUE;
    private int expectedPipeline = -1;
    private long settleUntilMs = Long.MIN_VALUE;
    private long nextId = 1;
    private Reject lastReject = Reject.NO_RESULT;

    /** Call on every pipeline switch / mode change. */
    public void expect(int pipelineIndex, long nowMs, long settleMs) {
        expectedPipeline = pipelineIndex;
        settleUntilMs = nowMs + settleMs;
    }

    public Reject lastReject() {
        return lastReject;
    }

    public CameraFrame accept(RawResult r, long nowMs, CameraMode mode, PipelineSpec spec,
                              PoseHistory poses, boolean turretOk) {
        if (r == null) {
            return reject(Reject.NO_RESULT);
        }
        if (r.chTimestampMs == lastTimestampMs) {
            return reject(Reject.NOT_NEW);
        }
        lastTimestampMs = r.chTimestampMs;
        if (r.pipelineIndex != expectedPipeline) {
            return reject(Reject.WRONG_PIPELINE);
        }
        if (r.stalenessMs > PerceptionConfig.MAX_STALENESS_MS) {
            return reject(Reject.STALE);
        }
        if (nowMs < settleUntilMs) {
            return reject(Reject.SETTLING);
        }
        if (mode == CameraMode.BALL && !turretOk) {
            return reject(Reject.TURRET);
        }
        long capture = r.chTimestampMs - Math.round(r.captureLatencyMs + r.targetingLatencyMs);
        Pose pose = poses.at(capture);
        if (pose == null) {
            return reject(Reject.NO_POSE);
        }
        ArrayList<Detection> balls = new ArrayList<Detection>();
        if (mode == CameraMode.BALL && spec != null) {
            if (spec.type == PipelineSpec.Type.NEURAL) {
                balls.addAll(r.detections);
            } else {
                for (Detection d : r.colourBlobs) {
                    balls.add(d.withClassId(spec.colourClass.classId));
                }
            }
        }
        lastReject = Reject.NONE;
        return new CameraFrame(nextId++, mode, mode == CameraMode.BALL ? spec : null, capture, pose,
                Collections.unmodifiableList(balls));
    }

    private CameraFrame reject(Reject why) {
        lastReject = why;
        return null;
    }
}
```

- [ ] **Step 4: Run the test to make sure it passes**

Run: `./gradlew :TeamCode:testDebugUnitTest --tests 'org.firstinspires.ftc.teamcode.perception.FrameGateTest'`
Expected: PASS (10 tests).

- [ ] **Step 5: Commit**

```bash
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/perception TeamCode/src/test/java/org/firstinspires/ftc/teamcode/perception/FrameGateTest.java
git commit -m "Add frame gating and ball pipeline cycling" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 7: Camera and ball tracking subsystems (hardware glue)

**Files:**
- Create: `MAIN/perception/CameraSubsystem.java`, `MAIN/perception/BallTrackingSubsystem.java`

**Interfaces:**
- Consumes: everything in `perception`
- Produces:
  - `CameraSubsystem()` (NextFTC `Subsystem`; gets hardware in `initialize()` via `ActiveOpMode.hardwareMap()`), `setPoseSource(PoseSource)`, `setTurretCheck(BooleanSupplier)`, `setMode(CameraMode)`, `modeCommand(CameraMode) -> Command`, `mode()`, `latestFrame() -> CameraFrame|null`, `latestRawResult() -> LLResult|null` (for other people's goal-mode code), `lastReject()`
  - `BallTrackingSubsystem(CameraSubsystem)`, `map() -> BallMap`

- [ ] **Step 1: Implement CameraSubsystem**

`MAIN/perception/CameraSubsystem.java`:

```java
package org.firstinspires.ftc.teamcode.perception;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

/**
 * Sole owner of the Limelight and its tilt servo. Publishes gated, pose-stamped frames;
 * feature subsystems pull latestFrame() and skip ids they have already used.
 */
public final class CameraSubsystem implements Subsystem {
    private Limelight3A limelight;
    private Servo tilt;
    private PoseSource poseSource;
    private BooleanSupplier turretOk = new BooleanSupplier() {
        @Override
        public boolean getAsBoolean() {
            return true;
        }
    };
    private final PoseHistory poses = new PoseHistory(PerceptionConfig.POSE_HISTORY_MS);
    private final FrameGate gate = new FrameGate();
    private PipelineCycle cycle;
    private CameraMode mode;
    private volatile CameraFrame latestFrame;
    private volatile LLResult latestRaw;

    public void setPoseSource(PoseSource source) {
        this.poseSource = source;
    }

    /** Optional: return false when the turret is not at the ball-mode angle. */
    public void setTurretCheck(BooleanSupplier check) {
        this.turretOk = check;
    }

    @Override
    public void initialize() {
        HardwareMap hw = ActiveOpMode.hardwareMap();
        limelight = hw.get(Limelight3A.class, PerceptionConfig.LIMELIGHT_NAME);
        tilt = PerceptionConfig.TILT_SERVO_NAME.isEmpty()
                ? null : hw.get(Servo.class, PerceptionConfig.TILT_SERVO_NAME);
        cycle = PipelineCycle.parse(PerceptionConfig.BALL_PIPELINES);
        mode = null;
        latestFrame = null;
        limelight.setPollRateHz(100);
        limelight.start();
        setMode(CameraMode.BALL);
    }

    public void setMode(CameraMode m) {
        if (m == mode || limelight == null) {
            return;
        }
        mode = m;
        if (tilt != null) {
            tilt.setPosition(m == CameraMode.BALL ? PerceptionConfig.TILT_BALL_POS : PerceptionConfig.TILT_GOAL_POS);
        }
        int index = m == CameraMode.BALL ? cycle.current().index : PerceptionConfig.GOAL_PIPELINE;
        limelight.pipelineSwitch(index);
        gate.expect(index, System.currentTimeMillis(), PerceptionConfig.MODE_SETTLE_MS);
        latestFrame = null;
    }

    public Command modeCommand(final CameraMode m) {
        return instant("Camera " + m, new Runnable() {
            @Override
            public void run() {
                setMode(m);
            }
        });
    }

    @Override
    public void periodic() {
        if (limelight == null) {
            return;
        }
        long now = System.currentTimeMillis();
        if (poseSource != null) {
            poses.add(now, poseSource.pose());
        }
        LLResult r = limelight.getLatestResult();
        latestRaw = r;
        if (r == null) {
            return;
        }
        PipelineSpec spec = mode == CameraMode.BALL ? cycle.current() : null;
        CameraFrame f = gate.accept(toRaw(r), now, mode, spec, poses, turretOk.getAsBoolean());
        if (f == null) {
            return;
        }
        latestFrame = f;
        if (mode == CameraMode.BALL && cycle.size() > 1) {
            PipelineSpec next = cycle.advance();
            limelight.pipelineSwitch(next.index);
            gate.expect(next.index, now, PerceptionConfig.PIPELINE_SETTLE_MS);
        }
    }

    static RawResult toRaw(LLResult r) {
        ArrayList<Detection> det = new ArrayList<Detection>();
        List<LLResultTypes.DetectorResult> dr = r.getDetectorResults();
        if (dr != null) {
            for (LLResultTypes.DetectorResult d : dr) {
                det.add(new Detection(d.getClassId(), d.getConfidence(), d.getTargetXDegrees(),
                        d.getTargetYDegrees(), Detection.aspectFromCorners(d.getTargetCorners())));
            }
        }
        ArrayList<Detection> col = new ArrayList<Detection>();
        List<LLResultTypes.ColorResult> cr = r.getColorResults();
        if (cr != null) {
            for (LLResultTypes.ColorResult c : cr) {
                col.add(new Detection(-1, 1.0, c.getTargetXDegrees(), c.getTargetYDegrees(),
                        Detection.aspectFromCorners(c.getTargetCorners())));
            }
        }
        return new RawResult(r.getControlHubTimeStamp(), r.getStaleness(), r.getPipelineIndex(),
                r.getCaptureLatency(), r.getTargetingLatency(), det, col);
    }

    public CameraMode mode() { return mode; }
    public CameraFrame latestFrame() { return latestFrame; }
    public LLResult latestRawResult() { return latestRaw; }
    public FrameGate.Reject lastReject() { return gate.lastReject(); }
}
```

- [ ] **Step 2: Implement BallTrackingSubsystem**

`MAIN/perception/BallTrackingSubsystem.java`:

```java
package org.firstinspires.ftc.teamcode.perception;

import java.util.ArrayList;
import java.util.List;

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
            map.update(now, f.robotPose, m, sightings(f, m));
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
```

- [ ] **Step 3: Compile**

Run: `./gradlew :TeamCode:compileDebugJavaWithJavac`
Expected: `BUILD SUCCESSFUL`.

- [ ] **Step 4: Run all tests (nothing broke)**

Run: `./gradlew :TeamCode:testDebugUnitTest`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/perception/CameraSubsystem.java TeamCode/src/main/java/org/firstinspires/ftc/teamcode/perception/BallTrackingSubsystem.java
git commit -m "Add camera and ball tracking subsystems" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 8: Field model and the strategy plug-in (CollectRequest)

**Files:**
- Create: `MAIN/collect/Alliance.java`, `MAIN/collect/CollectConfig.java`, `MAIN/collect/FieldRect.java`, `MAIN/collect/FieldFrame.java`, `MAIN/collect/FieldGeometry.java`, `MAIN/collect/BallSource.java`, `MAIN/collect/ZonePreset.java`, `MAIN/collect/CollectRequest.java`, `MAIN/collect/AllowedArea.java`
- Test: `TEST/collect/FieldModelTest.java`

**Interfaces:**
- Consumes: `BallClass`, `MapBall`, `Angles`
- Produces:
  - `Alliance {RED, BLUE}`, `ownNectar() -> BallClass`
  - `FieldRect(minX, minY, maxX, maxY)`, `FIELD`, `contains`, `intersect -> FieldRect|null`, `complementIn(FieldRect) -> List<FieldRect>`
  - `FieldFrame(double rotationDeg)`, `fromConfig()`, `toPedro(x, y) -> double[2]`, `toManual(x, y) -> double[2]`, `headingToPedro(rad)`
  - `FieldGeometry.HIVE`, `FLOWERS`, `FLOWER_RADIUS_IN`, `FIELD_CENTER`, `ownHalf(Alliance)`, `staticObstacles(FieldFrame) -> List<Zone>`, `rectZone(FieldRect, FieldFrame) -> Zone`
  - `BallSource` fields `name`, `kind`, `alliance`, `lookX`, `lookY`, `lookHeadingDeg`, `enabledByDefault`; `ALL`; `usableBy(Alliance)`
  - `ZonePreset {WHOLE_FIELD, OWN_HALF, NEAR_HALF, FAR_HALF, CUSTOM}`, `rect(Alliance)`, `next()`
  - `CollectRequest.builder(Alliance, boolean auto)` with `classes(...)`, `weight(BallClass, double)`, `zone(ZonePreset)`, `enableSource(String)`, `disableSource(String)`, `onlySources(String...)`, `useHints(boolean)`, `maxTargets(int)`, `timeoutSec(double)`, `endWhenFull(boolean)`, `build()`; public final fields `alliance`, `auto`, `classes`, `zone`, `sources`, `useHints`, `maxTargets`, `timeoutSec`, `endWhenFull`; `weight(BallClass)`
  - `AllowedArea(CollectRequest, FieldFrame)`, `rect()`, `containsPedro(x, y)`, `targetOk(MapBall)`, `outsideZones() -> List<Zone>`

- [ ] **Step 1: Write the failing test**

`TEST/collect/FieldModelTest.java`:

```java
package org.firstinspires.ftc.teamcode.collect;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.perception.BallClass;
import org.junit.Test;

import java.util.List;

public class FieldModelTest {
    @Test
    public void rectIntersectAndComplement() {
        FieldRect half = new FieldRect(0, 0, 72, 144);
        assertNull(half.intersect(new FieldRect(80, 0, 144, 144)));
        assertEquals(36, half.intersect(new FieldRect(36, 0, 144, 144)).minX, 1e-9);
        List<FieldRect> rest = half.complementIn(FieldRect.FIELD);
        assertEquals(1, rest.size());
        assertEquals(72, rest.get(0).minX, 1e-9);
        assertEquals(4, new FieldRect(10, 10, 20, 20).complementIn(FieldRect.FIELD).size());
    }

    @Test
    public void fieldFrameRotatesAboutTheCentreAndRoundTrips() {
        FieldFrame id = new FieldFrame(0);
        assertEquals(10, id.toPedro(10, 20)[0], 1e-9);
        FieldFrame q = new FieldFrame(90);
        double[] p = q.toPedro(144, 72);
        assertEquals(72, p[0], 1e-9);
        assertEquals(144, p[1], 1e-9);
        double[] back = q.toManual(p[0], p[1]);
        assertEquals(144, back[0], 1e-9);
        assertEquals(72, back[1], 1e-9);
        assertEquals(Math.PI / 2, q.headingToPedro(0), 1e-9);
    }

    @Test
    public void staticObstaclesAreTheHiveAndFourFlowers() {
        assertEquals(5, FieldGeometry.staticObstacles(new FieldFrame(0)).size());
    }

    @Test
    public void defaultRequestIsPollenPlusOwnNectarWithSafeSources() {
        CollectRequest r = CollectRequest.builder(Alliance.BLUE, false).build();
        assertTrue(r.classes.contains(BallClass.POLLEN));
        assertTrue(r.classes.contains(BallClass.BLUE_NECTAR));
        assertFalse(r.classes.contains(BallClass.RED_NECTAR));
        assertTrue(r.sources.contains("LOADING_ZONE_BLUE"));
        assertFalse(r.sources.contains("LOADING_ZONE_RED"));
        assertFalse(r.sources.contains("GARDEN_BLUE"));   // gardens score: off by default
        assertFalse(r.sources.contains("FLOWER_1"));      // until the intake can use retrieval openings
        assertEquals(0, r.timeoutSec, 1e-9);
        assertEquals(CollectConfig.AUTO_TIMEOUT_SEC, CollectRequest.builder(Alliance.BLUE, true).build().timeoutSec, 1e-9);
        assertEquals(1.0, r.weight(BallClass.POLLEN), 1e-9);
    }

    @Test
    public void autoRestrictsToOwnHalfWithAMidlineMargin() {
        CollectRequest r = CollectRequest.builder(Alliance.RED, true).build();
        AllowedArea a = new AllowedArea(r, new FieldFrame(0));
        assertTrue(a.containsPedro(30, 30));
        assertFalse(a.containsPedro(100, 30));
        assertEquals(1, a.outsideZones().size());
    }

    @Test
    public void teleopWholeFieldHasNoOutsideZones() {
        AllowedArea a = new AllowedArea(CollectRequest.builder(Alliance.RED, false).build(), new FieldFrame(0));
        assertTrue(a.containsPedro(100, 30));
        assertEquals(0, a.outsideZones().size());
    }

    @Test
    public void presetsCycle() {
        assertEquals(ZonePreset.OWN_HALF, ZonePreset.WHOLE_FIELD.next());
        assertEquals(ZonePreset.WHOLE_FIELD, ZonePreset.CUSTOM.next());
        assertEquals(72, ZonePreset.OWN_HALF.rect(Alliance.BLUE).minX, 1e-9);
    }
}
```

The midline margin on targets is tested with real `MapBall`s in Task 9.

- [ ] **Step 2: Run it to make sure it fails**

Run: `./gradlew :TeamCode:testDebugUnitTest --tests 'org.firstinspires.ftc.teamcode.collect.FieldModelTest'`
Expected: FAIL, "cannot find symbol: class FieldRect".

- [ ] **Step 3: Implement**

`MAIN/collect/Alliance.java`:

```java
package org.firstinspires.ftc.teamcode.collect;

import org.firstinspires.ftc.teamcode.perception.BallClass;

public enum Alliance {
    RED,
    BLUE;

    public BallClass ownNectar() {
        return this == RED ? BallClass.RED_NECTAR : BallClass.BLUE_NECTAR;
    }
}
```

`MAIN/collect/CollectConfig.java`:

```java
package org.firstinspires.ftc.teamcode.collect;

import com.acmerobotics.dashboard.config.Config;

/** Every collect/pathing tunable. Editable live from FTC Dashboard; read at point of use. */
@Config("Collect")
public final class CollectConfig {
    private CollectConfig() {
    }

    // frames
    public static double PEDRO_ROTATION_DEG = 0;   // manual frame -> Pedro frame, multiples of 90
    public static double DASH_ROTATION_DEG = 0;    // Pedro frame -> dashboard drawing

    // robot + intake (Weaver geometry)
    public static int INTAKE_CAPACITY = 4;         // only for CountingIntakeStatus; real intake reports its own
    public static double INTAKE_WIDTH_IN = 11.3;
    public static double ROBOT_LENGTH_IN = 18.0;
    public static double ROBOT_WIDTH_IN = 18.0;
    public static double CLEARANCE_IN = 2.0;
    public static int MAX_WEAVER_TARGETS = 7;      // Weaver's exact-ordering cap

    // zones
    public static double MIDLINE_MARGIN_IN = 4.0;
    public static double CUSTOM_MIN_X = 0, CUSTOM_MIN_Y = 0, CUSTOM_MAX_X = 144, CUSTOM_MAX_Y = 144;

    // run
    public static double AUTO_TIMEOUT_SEC = 20;
    public static double LOOK_DISTANCE_IN = 30;
    public static long LOOK_DWELL_MS = 400;
    public static double STICK_CANCEL = 0.1;

    // re-plan
    public static long REPLAN_COOLDOWN_MS = 300;
    public static double OFF_PATH_IN = 6.0;
    public static double OFF_PATH_DEG = 20.0;
    public static long STALL_MS = 500;
    public static double STALL_MOVE_IN = 1.0;
    public static double POSE_JUMP_IN = 4.0;
    public static double BETTER_BALL_MARGIN_IN = 6.0;
    public static double REACH_TOL_IN = 6.0;
    public static long PICKUP_GRACE_MS = 600;
    public static int MISS_LIMIT = 2;
}
```

`MAIN/collect/FieldRect.java`:

```java
package org.firstinspires.ftc.teamcode.collect;

import java.util.ArrayList;
import java.util.List;

/** Axis-aligned rectangle in the manual frame (inches). */
public final class FieldRect {
    public static final FieldRect FIELD = new FieldRect(0, 0, 144, 144);

    public final double minX;
    public final double minY;
    public final double maxX;
    public final double maxY;

    public FieldRect(double minX, double minY, double maxX, double maxY) {
        if (maxX < minX || maxY < minY) {
            throw new IllegalArgumentException("Empty rectangle");
        }
        this.minX = minX;
        this.minY = minY;
        this.maxX = maxX;
        this.maxY = maxY;
    }

    public boolean contains(double x, double y) {
        return x >= minX && x <= maxX && y >= minY && y <= maxY;
    }

    /** Overlap, or null if the rectangles don't overlap. */
    public FieldRect intersect(FieldRect o) {
        double a = Math.max(minX, o.minX);
        double b = Math.max(minY, o.minY);
        double c = Math.min(maxX, o.maxX);
        double d = Math.min(maxY, o.maxY);
        if (a > c || b > d) {
            return null;
        }
        return new FieldRect(a, b, c, d);
    }

    /** The parts of outer that are not this rectangle, as up to four bands. */
    public List<FieldRect> complementIn(FieldRect outer) {
        ArrayList<FieldRect> out = new ArrayList<FieldRect>();
        if (minX > outer.minX) {
            out.add(new FieldRect(outer.minX, outer.minY, minX, outer.maxY));
        }
        if (maxX < outer.maxX) {
            out.add(new FieldRect(maxX, outer.minY, outer.maxX, outer.maxY));
        }
        if (minY > outer.minY) {
            out.add(new FieldRect(minX, outer.minY, maxX, minY));
        }
        if (maxY < outer.maxY) {
            out.add(new FieldRect(minX, maxY, maxX, outer.maxY));
        }
        return out;
    }
}
```

`MAIN/collect/FieldFrame.java`:

```java
package org.firstinspires.ftc.teamcode.collect;

import org.firstinspires.ftc.teamcode.perception.Angles;

/**
 * Converts the manual frame (x along the audience side A->F, y away from the audience) into
 * the Pedro field frame: a rotation by quarter turns (CCW) about the field centre.
 */
public final class FieldFrame {
    private final int quarterTurns;

    public FieldFrame(double rotationDeg) {
        this.quarterTurns = (((int) Math.round(rotationDeg / 90.0)) % 4 + 4) % 4;
    }

    public static FieldFrame fromConfig() {
        return new FieldFrame(CollectConfig.PEDRO_ROTATION_DEG);
    }

    public double[] toPedro(double x, double y) {
        return rotate(x, y, quarterTurns);
    }

    public double[] toManual(double x, double y) {
        return rotate(x, y, (4 - quarterTurns) % 4);
    }

    public double headingToPedro(double rad) {
        return Angles.wrap(rad + quarterTurns * Math.PI / 2);
    }

    private static double[] rotate(double x, double y, int q) {
        double dx = x - 72;
        double dy = y - 72;
        for (int i = 0; i < q; i++) {
            double t = dx;
            dx = -dy;
            dy = t;
        }
        return new double[]{72 + dx, 72 + dy};
    }
}
```

`MAIN/collect/FieldGeometry.java`:

```java
package org.firstinspires.ftc.teamcode.collect;

import com.skeletonarmy.marrow.zones.CircleZone;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.PolygonZone;
import com.skeletonarmy.marrow.zones.Zone;

import java.util.ArrayList;
import java.util.List;

/**
 * BIOBUZZ field geometry in the manual frame (manual TU08 section 9, Fig. 9-2/9-4).
 * Positions are read off the drawings: verify on a real field.
 */
public final class FieldGeometry {
    private FieldGeometry() {
    }

    public static final double FIELD_CENTER = 72.0;
    /** Hive frame base, 49.46 x 38.95 in, centred. */
    public static final FieldRect HIVE = new FieldRect(47.0, 52.5, 97.0, 91.5);
    /** Flower centres on the perimeter walls. */
    public static final double[][] FLOWERS = {{48, 144}, {96, 0}, {0, 48}, {144, 96}};
    public static final double FLOWER_RADIUS_IN = 4.0;

    /** AUTO sides (manual G402): columns A-C red, D-F blue. */
    public static FieldRect ownHalf(Alliance a) {
        return a == Alliance.RED ? new FieldRect(0, 0, 72, 144) : new FieldRect(72, 0, 144, 144);
    }

    public static List<Zone> staticObstacles(FieldFrame f) {
        ArrayList<Zone> out = new ArrayList<Zone>();
        out.add(rectZone(HIVE, f));
        for (double[] c : FLOWERS) {
            double[] p = f.toPedro(c[0], c[1]);
            out.add(new CircleZone(new Point(p[0], p[1]), FLOWER_RADIUS_IN));
        }
        return out;
    }

    public static Zone rectZone(FieldRect r, FieldFrame f) {
        double[] a = f.toPedro(r.minX, r.minY);
        double[] b = f.toPedro(r.maxX, r.minY);
        double[] c = f.toPedro(r.maxX, r.maxY);
        double[] d = f.toPedro(r.minX, r.maxY);
        return new PolygonZone(new Point(a[0], a[1]), new Point(b[0], b[1]),
                new Point(c[0], c[1]), new Point(d[0], d[1]));
    }
}
```

`MAIN/collect/BallSource.java`:

```java
package org.firstinspires.ftc.teamcode.collect;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * Known places balls sit (manual section 10), with a pose to look from (manual frame, heading in
 * degrees CCW from +x). Adding a spot = one line in ALL.
 */
public final class BallSource {
    public enum Kind { GARDEN, LOADING_ZONE, FLOWER, CUSTOM }

    public final String name;
    public final Kind kind;
    public final Alliance alliance;   // null = either alliance
    public final double lookX;
    public final double lookY;
    public final double lookHeadingDeg;
    public final boolean enabledByDefault;

    public BallSource(String name, Kind kind, Alliance alliance, double lookX, double lookY,
                      double lookHeadingDeg, boolean enabledByDefault) {
        this.name = name;
        this.kind = kind;
        this.alliance = alliance;
        this.lookX = lookX;
        this.lookY = lookY;
        this.lookHeadingDeg = lookHeadingDeg;
        this.enabledByDefault = enabledByDefault;
    }

    public boolean usableBy(Alliance a) {
        return alliance == null || alliance == a;
    }

    /** Gardens are scoring areas, so they're off by default; flowers stay off until the intake can use the retrieval opening. */
    public static final List<BallSource> ALL = Collections.unmodifiableList(Arrays.asList(
            new BallSource("GARDEN_RED", Kind.GARDEN, Alliance.RED, 14, 32, -90, false),
            new BallSource("GARDEN_BLUE", Kind.GARDEN, Alliance.BLUE, 130, 112, 90, false),
            new BallSource("LOADING_ZONE_RED", Kind.LOADING_ZONE, Alliance.RED, 36, 108, 180, true),
            new BallSource("LOADING_ZONE_BLUE", Kind.LOADING_ZONE, Alliance.BLUE, 108, 36, 0, true),
            new BallSource("FLOWER_1", Kind.FLOWER, null, 48, 120, 90, false),
            new BallSource("FLOWER_2", Kind.FLOWER, null, 96, 24, -90, false),
            new BallSource("FLOWER_3", Kind.FLOWER, null, 24, 48, 180, false),
            new BallSource("FLOWER_4", Kind.FLOWER, null, 120, 96, 0, false)));
}
```

`MAIN/collect/ZonePreset.java`:

```java
package org.firstinspires.ftc.teamcode.collect;

/** Where our robot may collect (manual frame). Chosen on the gamepad during init. */
public enum ZonePreset {
    WHOLE_FIELD,
    OWN_HALF,
    NEAR_HALF,
    FAR_HALF,
    CUSTOM;

    public FieldRect rect(Alliance a) {
        switch (this) {
            case OWN_HALF:
                return FieldGeometry.ownHalf(a);
            case NEAR_HALF:
                return new FieldRect(0, 0, 144, 72);
            case FAR_HALF:
                return new FieldRect(0, 72, 144, 144);
            case CUSTOM:
                return new FieldRect(CollectConfig.CUSTOM_MIN_X, CollectConfig.CUSTOM_MIN_Y,
                        CollectConfig.CUSTOM_MAX_X, CollectConfig.CUSTOM_MAX_Y);
            default:
                return FieldRect.FIELD;
        }
    }

    public ZonePreset next() {
        return values()[(ordinal() + 1) % values().length];
    }
}
```

`MAIN/collect/CollectRequest.java`:

```java
package org.firstinspires.ftc.teamcode.collect;

import org.firstinspires.ftc.teamcode.perception.BallClass;

import java.util.Collections;
import java.util.EnumMap;
import java.util.EnumSet;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

/** Everything strategic about one collect run. Defaults allow everything that is safe. */
public final class CollectRequest {
    public final Alliance alliance;
    public final boolean auto;
    public final Set<BallClass> classes;
    public final ZonePreset zone;
    public final Set<String> sources;
    public final boolean useHints;
    public final int maxTargets;          // <= 0 = intake capacity
    public final double timeoutSec;       // <= 0 = none
    public final boolean endWhenFull;
    private final Map<BallClass, Double> weights;

    private CollectRequest(Builder b) {
        this.alliance = b.alliance;
        this.auto = b.auto;
        this.classes = Collections.unmodifiableSet(EnumSet.copyOf(b.classes));
        this.zone = b.zone;
        this.sources = Collections.unmodifiableSet(new HashSet<String>(b.sources));
        this.useHints = b.useHints;
        this.maxTargets = b.maxTargets;
        this.timeoutSec = b.timeoutSec;
        this.endWhenFull = b.endWhenFull;
        this.weights = new EnumMap<BallClass, Double>(b.weights);
    }

    public double weight(BallClass c) {
        Double w = weights.get(c);
        return w == null ? 1.0 : w;
    }

    public static Builder builder(Alliance alliance, boolean auto) {
        return new Builder(alliance, auto);
    }

    public static final class Builder {
        private final Alliance alliance;
        private final boolean auto;
        private EnumSet<BallClass> classes;
        private final EnumMap<BallClass, Double> weights = new EnumMap<BallClass, Double>(BallClass.class);
        private ZonePreset zone = ZonePreset.WHOLE_FIELD;
        private final Set<String> sources = new HashSet<String>();
        private boolean useHints = true;
        private int maxTargets = -1;
        private double timeoutSec;
        private boolean endWhenFull = true;

        private Builder(Alliance alliance, boolean auto) {
            this.alliance = alliance;
            this.auto = auto;
            this.classes = EnumSet.of(BallClass.POLLEN, alliance.ownNectar());
            for (BallSource s : BallSource.ALL) {
                if (s.enabledByDefault && s.usableBy(alliance)) {
                    sources.add(s.name);
                }
            }
            this.timeoutSec = auto ? CollectConfig.AUTO_TIMEOUT_SEC : 0;
        }

        public Builder classes(BallClass... cs) {
            classes = EnumSet.noneOf(BallClass.class);
            Collections.addAll(classes, cs);
            return this;
        }

        public Builder weight(BallClass c, double w) {
            weights.put(c, w);
            return this;
        }

        public Builder zone(ZonePreset z) {
            zone = z;
            return this;
        }

        public Builder enableSource(String name) {
            sources.add(name);
            return this;
        }

        public Builder disableSource(String name) {
            sources.remove(name);
            return this;
        }

        public Builder onlySources(String... names) {
            sources.clear();
            Collections.addAll(sources, names);
            return this;
        }

        public Builder useHints(boolean v) {
            useHints = v;
            return this;
        }

        public Builder maxTargets(int n) {
            maxTargets = n;
            return this;
        }

        public Builder timeoutSec(double s) {
            timeoutSec = s;
            return this;
        }

        public Builder endWhenFull(boolean v) {
            endWhenFull = v;
            return this;
        }

        public CollectRequest build() {
            return new CollectRequest(this);
        }
    }
}
```

`MAIN/collect/AllowedArea.java`:

```java
package org.firstinspires.ftc.teamcode.collect;

import com.skeletonarmy.marrow.zones.Zone;

import org.firstinspires.ftc.teamcode.perception.MapBall;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/** Where this run may collect: the zone preset, intersected with our half in AUTO (G402). */
public final class AllowedArea {
    private final FieldRect rect;      // manual frame; null = nowhere
    private final boolean auto;
    private final FieldFrame frame;

    public AllowedArea(CollectRequest req, FieldFrame frame) {
        FieldRect r = req.zone.rect(req.alliance);
        if (req.auto) {
            r = r.intersect(FieldGeometry.ownHalf(req.alliance));
        }
        this.rect = r;
        this.auto = req.auto;
        this.frame = frame;
    }

    public FieldRect rect() {
        return rect;
    }

    public boolean containsPedro(double x, double y) {
        if (rect == null) {
            return false;
        }
        double[] m = frame.toManual(x, y);
        return rect.contains(m[0], m[1]);
    }

    /** Inside the area and, in AUTO, clear of the midline by the margin plus the ball's uncertainty. */
    public boolean targetOk(MapBall b) {
        if (!containsPedro(b.x(), b.y())) {
            return false;
        }
        if (!auto) {
            return true;
        }
        double[] m = frame.toManual(b.x(), b.y());
        return Math.abs(m[0] - FieldGeometry.FIELD_CENTER) >= CollectConfig.MIDLINE_MARGIN_IN + b.sigma();
    }

    /** The rest of the field as Weaver obstacles. */
    public List<Zone> outsideZones() {
        if (rect == null) {
            return Collections.emptyList();
        }
        ArrayList<Zone> out = new ArrayList<Zone>();
        for (FieldRect r : rect.complementIn(FieldRect.FIELD)) {
            out.add(FieldGeometry.rectZone(r, frame));
        }
        return out;
    }
}
```

- [ ] **Step 4: Run the test to make sure it passes**

Run: `./gradlew :TeamCode:testDebugUnitTest --tests 'org.firstinspires.ftc.teamcode.collect.FieldModelTest'`
Expected: PASS (7 tests).

- [ ] **Step 5: Commit**

```bash
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/collect TeamCode/src/test/java/org/firstinspires/ftc/teamcode/collect
git commit -m "Add field model, zones and CollectRequest" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 9: Collect planner (target choice + Weaver)

**Files:**
- Create: `MAIN/collect/ObstacleProvider.java`, `MAIN/collect/CollectPlanner.java`
- Test: `TEST/collect/MapSeeds.java` (shared test helper), `TEST/collect/CollectPlannerTest.java`

**Interfaces:**
- Consumes: `BallMap`, `MapBall`, `Sighting`, `CollectRequest`, `AllowedArea`, `FieldGeometry`, `FieldFrame`, `CollectConfig`
- Produces:
  - `interface ObstacleProvider { List<Zone> obstacles(); }`
  - `CollectPlanner(FieldFrame)`, `static applyWeaverConfig()`
  - `chooseTargets(Pose robot, List<MapBall> candidates, CollectRequest req, AllowedArea area, int capacityLeft, Set<Integer> skip) -> List<MapBall>`
  - `obstacles(Pose robot, AllowedArea area, List<ObstacleProvider>) -> List<Zone>`
  - `plan(Pose robot, List<MapBall> targets, List<Zone> obstacles) -> CollectPlanner.Plan|null`
  - `planTo(Pose robot, Pose dest, List<Zone> obstacles) -> CollectPlanner.Plan|null`
  - `CollectPlanner.Plan` fields `path` (Pedro `Path`), `targets` (`List<MapBall>`, Weaver order), `preview` (`List<Point>`)
  - `lastError() -> String`
  - Test helper `MapSeeds.confirmed(BallMap, Object... classXY)`: seeds confirmed, precise, high-quality balls

- [ ] **Step 1: Write the test helper and failing test**

`TEST/collect/MapSeeds.java`:

```java
package org.firstinspires.ftc.teamcode.collect;

import org.firstinspires.ftc.teamcode.perception.BallClass;
import org.firstinspires.ftc.teamcode.perception.BallMap;
import org.firstinspires.ftc.teamcode.perception.Sighting;

import java.util.ArrayList;

/** Seeds confirmed balls: confirmed(map, BallClass.POLLEN, 40.0, 20.0, BallClass.RED_NECTAR, 60.0, 30.0, ...). */
final class MapSeeds {
    private MapSeeds() {
    }

    static void confirmed(BallMap map, Object... classXY) {
        seed(map, 1.0, 0.9, false, classXY);
    }

    static void hints(BallMap map, Object... classXY) {
        seed(map, 6.0, 0.25, false, classXY);
    }

    static void maybeTwo(BallMap map, Object... classXY) {
        seed(map, 1.0, 0.9, true, classXY);
    }

    private static void seed(BallMap map, double sigma, double q, boolean two, Object... classXY) {
        ArrayList<Sighting> ss = new ArrayList<Sighting>();
        for (int i = 0; i < classXY.length; i += 3) {
            ss.add(new Sighting((BallClass) classXY[i], (Double) classXY[i + 1], (Double) classXY[i + 2],
                    30, sigma, q, two));
        }
        map.update(0, null, null, ss);
        map.update(10, null, null, ss);
    }
}
```

Call it with `Double` literals (`40.0`, not `40`), because the varargs are `Object`.

`TEST/collect/CollectPlannerTest.java`:

```java
package org.firstinspires.ftc.teamcode.collect;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;

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
        CollectRequest req = CollectRequest.builder(Alliance.RED, false).weight(BallClass.RED_NECTAR, 3.0).build();
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
}
```

- [ ] **Step 2: Run it to make sure it fails**

Run: `./gradlew :TeamCode:testDebugUnitTest --tests 'org.firstinspires.ftc.teamcode.collect.CollectPlannerTest'`
Expected: FAIL, "cannot find symbol: class CollectPlanner".

- [ ] **Step 3: Implement**

`MAIN/collect/ObstacleProvider.java`:

```java
package org.firstinspires.ftc.teamcode.collect;

import com.skeletonarmy.marrow.zones.Zone;

import java.util.List;

/** Extra Weaver obstacles, read at every re-plan (e.g. robots from someone's robot tracker). Pedro frame. */
public interface ObstacleProvider {
    List<Zone> obstacles();
}
```

`MAIN/collect/CollectPlanner.java`:

```java
package org.firstinspires.ftc.teamcode.collect;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.skeletonarmy.marrow.weaver.HeadingPolicy;
import com.skeletonarmy.marrow.weaver.PathConfig;
import com.skeletonarmy.marrow.weaver.PathResult;
import com.skeletonarmy.marrow.weaver.PedroPathingConverter;
import com.skeletonarmy.marrow.weaver.Weaver;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.Zone;

import org.firstinspires.ftc.teamcode.perception.MapBall;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Set;

/** Chooses targets from the ball map and turns them into a Weaver route and a Pedro path. */
public final class CollectPlanner {
    public static final class Plan {
        public final Path path;
        public final List<MapBall> targets;
        public final List<Point> preview;

        Plan(Path path, List<MapBall> targets, List<Point> preview) {
            this.path = path;
            this.targets = targets;
            this.preview = preview;
        }
    }

    private final FieldFrame frame;
    private String lastError = "";

    public CollectPlanner(FieldFrame frame) {
        this.frame = frame;
    }

    /** Pushes our robot/intake geometry into Weaver's global config. */
    public static void applyWeaverConfig() {
        Weaver.setConfig(new PathConfig()
                .intakeWidth(CollectConfig.INTAKE_WIDTH_IN)
                .robotWidth(CollectConfig.ROBOT_LENGTH_IN)    // Weaver: along the forward axis
                .robotHeight(CollectConfig.ROBOT_WIDTH_IN)    // Weaver: across the forward axis
                .clearance(CollectConfig.CLEARANCE_IN));
    }

    public List<MapBall> chooseTargets(Pose robot, List<MapBall> candidates, final CollectRequest req,
                                       AllowedArea area, int capacityLeft, Set<Integer> skip) {
        int budget = capacityLeft;
        if (req.maxTargets > 0) {
            budget = Math.min(budget, req.maxTargets);
        }
        ArrayList<MapBall> pool = new ArrayList<MapBall>();
        for (MapBall b : candidates) {
            if (b.isTarget() && req.classes.contains(b.ballClass()) && !skip.contains(b.id()) && area.targetOk(b)) {
                pool.add(b);
            }
        }
        final double rx = robot.x();
        final double ry = robot.y();
        Collections.sort(pool, new Comparator<MapBall>() {
            @Override
            public int compare(MapBall a, MapBall b) {
                return Double.compare(score(a), score(b));
            }

            private double score(MapBall m) {
                return m.distanceTo(rx, ry) / req.weight(m.ballClass());
            }
        });
        ArrayList<MapBall> out = new ArrayList<MapBall>();
        int used = 0;
        for (MapBall b : pool) {
            if (used >= budget || out.size() >= CollectConfig.MAX_WEAVER_TARGETS) {
                break;
            }
            out.add(b);
            used += b.maybeTwo() ? 2 : 1;
        }
        return out;
    }

    /** Field obstacles + the outside of our area (only when we are inside it) + any providers. */
    public List<Zone> obstacles(Pose robot, AllowedArea area, List<ObstacleProvider> providers) {
        ArrayList<Zone> z = new ArrayList<Zone>(FieldGeometry.staticObstacles(frame));
        if (area.containsPedro(robot.x(), robot.y())) {
            z.addAll(area.outsideZones());
        }
        for (ObstacleProvider p : providers) {
            z.addAll(p.obstacles());
        }
        return z;
    }

    public Plan plan(Pose robot, List<MapBall> targets, List<Zone> obstacles) {
        if (targets.isEmpty()) {
            lastError = "no targets";
            return null;
        }
        ArrayList<Point> points = new ArrayList<Point>();
        for (MapBall b : targets) {
            points.add(new Point(b.x(), b.y()));
        }
        try {
            PathResult r = Weaver.builder()
                    .start(PedroPathingConverter.toPoint(robot))
                    .targets(points)
                    .obstacles(obstacles)
                    .build();
            ArrayList<MapBall> visited = new ArrayList<MapBall>();
            ArrayList<MapBall> remaining = new ArrayList<MapBall>(targets);
            for (Point p : r.getVisitedTargets()) {
                MapBall best = null;
                double bestD = 0.5;
                for (MapBall b : remaining) {
                    double d = b.distanceTo(p.getX(), p.getY());
                    if (d <= bestD) {
                        best = b;
                        bestD = d;
                    }
                }
                if (best != null) {
                    visited.add(best);
                    remaining.remove(best);
                }
            }
            if (visited.isEmpty()) {
                lastError = "Weaver reached no targets";
                return null;
            }
            return new Plan(PedroPathingConverter.toPath(r), visited, r.getPath().sample(8));
        } catch (RuntimeException e) {
            lastError = e.toString();
            return null;
        }
    }

    /** A path to a look pose, arriving with its heading. */
    public Plan planTo(Pose robot, Pose dest, List<Zone> obstacles) {
        try {
            PathResult r = Weaver.builder()
                    .start(PedroPathingConverter.toPoint(robot))
                    .end(new Point(dest.x(), dest.y(), dest.heading()), HeadingPolicy.LOCKED)
                    .obstacles(obstacles)
                    .build();
            return new Plan(PedroPathingConverter.toPath(r), Collections.<MapBall>emptyList(), r.getPath().sample(8));
        } catch (RuntimeException e) {
            lastError = e.toString();
            return null;
        }
    }

    public String lastError() {
        return lastError;
    }
}
```

- [ ] **Step 4: Run the test to make sure it passes**

Run: `./gradlew :TeamCode:testDebugUnitTest --tests 'org.firstinspires.ftc.teamcode.collect.CollectPlannerTest'`
Expected: PASS (8 tests). If `planVisitsTargetsAndProducesAPedroPath` fails with Weaver dropping a target, print `p.targets`/`planner.lastError()` and check that the target isn't inside an obstacle plus the robot footprint (`excludeBlockedTargets`). Move the test ball, not the planner.

- [ ] **Step 5: Commit**

```bash
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/collect TeamCode/src/test/java/org/firstinspires/ftc/teamcode/collect
git commit -m "Add collect planner over Weaver" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 10: Search planner (hints, then ball sources)

**Files:**
- Create: `MAIN/collect/SearchPlanner.java`
- Test: `TEST/collect/SearchPlannerTest.java`

**Interfaces:**
- Consumes: `MapBall`, `CollectRequest`, `AllowedArea`, `BallSource`, `FieldFrame`, `CollectConfig.LOOK_DISTANCE_IN`
- Produces: `SearchPlanner(FieldFrame)`, `nextLook(Pose robot, List<MapBall> hints, CollectRequest, AllowedArea, Set<String> visited) -> SearchPlanner.Look|null`; `Look` fields `pose` (Pedro `Pose`), `key` (`"hint:<id>"` or the source name)

- [ ] **Step 1: Write the failing test**

`TEST/collect/SearchPlannerTest.java`:

```java
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
```

- [ ] **Step 2: Run it to make sure it fails**

Run: `./gradlew :TeamCode:testDebugUnitTest --tests 'org.firstinspires.ftc.teamcode.collect.SearchPlannerTest'`
Expected: FAIL, "cannot find symbol: class SearchPlanner".

- [ ] **Step 3: Implement**

`MAIN/collect/SearchPlanner.java`:

```java
package org.firstinspires.ftc.teamcode.collect;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.perception.MapBall;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Set;

/** With no targets: look at the best hint up close, then visit known ball sources, nearest first. */
public final class SearchPlanner {
    public static final class Look {
        public final Pose pose;
        public final String key;

        Look(Pose pose, String key) {
            this.pose = pose;
            this.key = key;
        }
    }

    private final FieldFrame frame;

    public SearchPlanner(FieldFrame frame) {
        this.frame = frame;
    }

    public Look nextLook(final Pose robot, List<MapBall> hints, CollectRequest req, AllowedArea area,
                         Set<String> visited) {
        if (req.useHints) {
            ArrayList<MapBall> hs = new ArrayList<MapBall>();
            for (MapBall b : hints) {
                if (req.classes.contains(b.ballClass()) && area.targetOk(b) && !visited.contains("hint:" + b.id())) {
                    hs.add(b);
                }
            }
            Collections.sort(hs, new Comparator<MapBall>() {
                @Override
                public int compare(MapBall a, MapBall b) {
                    int q = Double.compare(b.quality(), a.quality());
                    return q != 0 ? q : Double.compare(a.distanceTo(robot.x(), robot.y()),
                            b.distanceTo(robot.x(), robot.y()));
                }
            });
            for (MapBall b : hs) {
                double dx = b.x() - robot.x();
                double dy = b.y() - robot.y();
                double d = Math.hypot(dx, dy);
                double heading = Math.atan2(dy, dx);
                double back = Math.min(CollectConfig.LOOK_DISTANCE_IN, d);
                double vx = b.x() - back * Math.cos(heading);
                double vy = b.y() - back * Math.sin(heading);
                if (area.containsPedro(vx, vy)) {
                    return new Look(new Pose(vx, vy, heading), "hint:" + b.id());
                }
            }
        }
        Look best = null;
        double bestD = Double.MAX_VALUE;
        for (BallSource s : BallSource.ALL) {
            if (!req.sources.contains(s.name) || !s.usableBy(req.alliance) || visited.contains(s.name)) {
                continue;
            }
            double[] p = frame.toPedro(s.lookX, s.lookY);
            if (!area.containsPedro(p[0], p[1])) {
                continue;
            }
            double d = Math.hypot(p[0] - robot.x(), p[1] - robot.y());
            if (d < bestD) {
                bestD = d;
                best = new Look(new Pose(p[0], p[1], frame.headingToPedro(Math.toRadians(s.lookHeadingDeg))), s.name);
            }
        }
        return best;
    }
}
```

- [ ] **Step 4: Run the test to make sure it passes**

Run: `./gradlew :TeamCode:testDebugUnitTest --tests 'org.firstinspires.ftc.teamcode.collect.SearchPlannerTest'`
Expected: PASS (4 tests).

- [ ] **Step 5: Commit**

```bash
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/collect/SearchPlanner.java TeamCode/src/test/java/org/firstinspires/ftc/teamcode/collect/SearchPlannerTest.java
git commit -m "Add search planner for hints and ball sources" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 11: Re-plan policy

**Files:**
- Create: `MAIN/collect/ReplanPolicy.java`
- Test: `TEST/collect/ReplanPolicyTest.java`

**Interfaces:**
- Consumes: `CollectConfig`
- Produces: `ReplanPolicy()`, `reset(long nowMs)`, `check(long nowMs, ReplanPolicy.Inputs) -> ReplanPolicy.Reason`; `Reason {NONE, POSE_JUMP, PATH_DONE, TARGET_GONE, UNEXPECTED_PICKUP, MISSED_TARGET, OFF_PATH, STALLED, BETTER_BALL}`; `Inputs` public mutable fields `pathDone`, `targetGone`, `betterBall`, `unexpectedPickup`, `missedTarget`, `poseJump`, `following` (booleans), `offPathIn`, `headingErrDeg` (doubles), `pose` (`Pose`)

- [ ] **Step 1: Write the failing test**

`TEST/collect/ReplanPolicyTest.java`:

```java
package org.firstinspires.ftc.teamcode.collect;

import static org.junit.Assert.assertEquals;

import com.pedropathing.math.Pose;

import org.junit.Before;
import org.junit.Test;

public class ReplanPolicyTest {
    private ReplanPolicy policy;
    private ReplanPolicy.Inputs in;

    @Before
    public void setUp() {
        policy = new ReplanPolicy();
        policy.reset(0);
        in = new ReplanPolicy.Inputs();
        in.following = true;
        in.pose = new Pose(0, 0, 0);
    }

    @Test
    public void cooldownHoldsBackEvents() {
        in.targetGone = true;
        assertEquals(ReplanPolicy.Reason.NONE, policy.check(100, in));
        assertEquals(ReplanPolicy.Reason.TARGET_GONE, policy.check(300, in));
        assertEquals(ReplanPolicy.Reason.NONE, policy.check(400, in));
    }

    @Test
    public void pathDoneAndPoseJumpIgnoreTheCooldown() {
        in.pathDone = true;
        assertEquals(ReplanPolicy.Reason.PATH_DONE, policy.check(10, in));
        in.pathDone = false;
        in.poseJump = true;
        assertEquals(ReplanPolicy.Reason.POSE_JUMP, policy.check(20, in));
    }

    @Test
    public void offPathByDistanceOrHeading() {
        in.offPathIn = 7;
        assertEquals(ReplanPolicy.Reason.OFF_PATH, policy.check(400, in));
        in.offPathIn = 0;
        in.headingErrDeg = 25;
        assertEquals(ReplanPolicy.Reason.OFF_PATH, policy.check(800, in));
    }

    @Test
    public void stallFiresOnlyWhenNotMoving() {
        assertEquals(ReplanPolicy.Reason.NONE, policy.check(400, in));
        assertEquals(ReplanPolicy.Reason.NONE, policy.check(700, in));
        assertEquals(ReplanPolicy.Reason.STALLED, policy.check(900, in));
        in.pose = new Pose(0, 0, 0);
        assertEquals(ReplanPolicy.Reason.NONE, policy.check(1300, in));
        in.pose = new Pose(5, 0, 0);
        assertEquals(ReplanPolicy.Reason.NONE, policy.check(1500, in));
        in.pose = new Pose(10, 0, 0);
        assertEquals(ReplanPolicy.Reason.NONE, policy.check(1900, in));
    }

    @Test
    public void eventPriorityOrder() {
        in.betterBall = true;
        in.missedTarget = true;
        assertEquals(ReplanPolicy.Reason.MISSED_TARGET, policy.check(400, in));
    }
}
```

Stall walk-through: anchored at 400; 700 − 400 = 300 < 500; at 900, 500 ≥ 500 → STALLED, which resets the anchor. At 1300 the cooldown has passed and the robot hasn't moved, but the anchor was reset at 900, so it re-anchors at 1300. At 1500 it has moved 5 in → re-anchor; at 1900 it has moved 5 in → re-anchor. So the result is NONE.

- [ ] **Step 2: Run it to make sure it fails**

Run: `./gradlew :TeamCode:testDebugUnitTest --tests 'org.firstinspires.ftc.teamcode.collect.ReplanPolicyTest'`
Expected: FAIL, "cannot find symbol: class ReplanPolicy".

- [ ] **Step 3: Implement**

`MAIN/collect/ReplanPolicy.java`:

```java
package org.firstinspires.ftc.teamcode.collect;

import com.pedropathing.math.Pose;

/** Event-driven re-plan decisions with a cooldown (spec Q7). */
public final class ReplanPolicy {
    public enum Reason {
        NONE, POSE_JUMP, PATH_DONE, TARGET_GONE, UNEXPECTED_PICKUP, MISSED_TARGET, OFF_PATH, STALLED, BETTER_BALL
    }

    public static final class Inputs {
        public boolean pathDone;
        public boolean targetGone;
        public boolean betterBall;
        public boolean unexpectedPickup;
        public boolean missedTarget;
        public boolean poseJump;
        public boolean following;
        public double offPathIn;
        public double headingErrDeg;
        public Pose pose;
    }

    private long lastReplanMs;
    private long stallSinceMs = -1;
    private double anchorX;
    private double anchorY;

    public void reset(long nowMs) {
        lastReplanMs = nowMs;
        stallSinceMs = -1;
    }

    public Reason check(long nowMs, Inputs in) {
        Reason r = decide(nowMs, in);
        if (r != Reason.NONE) {
            lastReplanMs = nowMs;
            stallSinceMs = -1;
        }
        return r;
    }

    private Reason decide(long nowMs, Inputs in) {
        if (in.poseJump) {
            return Reason.POSE_JUMP;
        }
        if (in.pathDone) {
            return Reason.PATH_DONE;
        }
        boolean stalled = updateStall(nowMs, in);
        if (nowMs - lastReplanMs < CollectConfig.REPLAN_COOLDOWN_MS) {
            return Reason.NONE;
        }
        if (in.targetGone) {
            return Reason.TARGET_GONE;
        }
        if (in.unexpectedPickup) {
            return Reason.UNEXPECTED_PICKUP;
        }
        if (in.missedTarget) {
            return Reason.MISSED_TARGET;
        }
        if (in.offPathIn > CollectConfig.OFF_PATH_IN || in.headingErrDeg > CollectConfig.OFF_PATH_DEG) {
            return Reason.OFF_PATH;
        }
        if (stalled) {
            return Reason.STALLED;
        }
        if (in.betterBall) {
            return Reason.BETTER_BALL;
        }
        return Reason.NONE;
    }

    private boolean updateStall(long nowMs, Inputs in) {
        if (!in.following || in.pose == null) {
            stallSinceMs = -1;
            return false;
        }
        if (stallSinceMs < 0
                || Math.hypot(in.pose.x() - anchorX, in.pose.y() - anchorY) > CollectConfig.STALL_MOVE_IN) {
            anchorX = in.pose.x();
            anchorY = in.pose.y();
            stallSinceMs = nowMs;
            return false;
        }
        return nowMs - stallSinceMs >= CollectConfig.STALL_MS;
    }
}
```

- [ ] **Step 4: Run the test to make sure it passes**

Run: `./gradlew :TeamCode:testDebugUnitTest --tests 'org.firstinspires.ftc.teamcode.collect.ReplanPolicyTest'`
Expected: PASS (5 tests).

- [ ] **Step 5: Commit**

```bash
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/collect/ReplanPolicy.java TeamCode/src/test/java/org/firstinspires/ftc/teamcode/collect/ReplanPolicyTest.java
git commit -m "Add event-driven re-plan policy" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 12: Collect brain (the state machine)

**Files:**
- Create: `MAIN/collect/IntakeStatus.java`, `MAIN/collect/PathFollower.java`, `MAIN/collect/CollectResult.java`, `MAIN/collect/CollectBrain.java`
- Test: `TEST/collect/CollectBrainTest.java`

**Interfaces:**
- Consumes: `CollectPlanner`, `SearchPlanner`, `ReplanPolicy`, `AllowedArea`, `BallMap`, `PerceptionConfig.INTAKE_REACH_IN`, `Angles`
- Produces:
  - `interface IntakeStatus { int ballCount(); int capacity(); default boolean isFull() }`
  - `interface PathFollower { void follow(Path); boolean isBusy(); Pose pose(); Pose closestPose(); void stop(); }` (`closestPose()` may be null)
  - `enum CollectResult { FULL, NO_TARGETS, TIMEOUT, INTERRUPTED }`
  - `CollectBrain(CollectRequest, BallMap, PathFollower, IntakeStatus, FieldFrame, List<ObstacleProvider>)`, `start(long)`, `update(long)`, `stop()`, `isDone()`, `result()`, `plannedTargets()`, `preview()`, `lastReason()`, `replanCount()`, `skipped()`, `state()` (String), `area()`

- [ ] **Step 1: Write the failing test**

`TEST/collect/CollectBrainTest.java`:

```java
package org.firstinspires.ftc.teamcode.collect;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.perception.BallClass;
import org.firstinspires.ftc.teamcode.perception.BallMap;
import org.junit.Before;
import org.junit.Test;

import java.util.Collections;

public class CollectBrainTest {
    static final class FakeFollower implements PathFollower {
        Pose pose;
        boolean busy;
        boolean stopped;
        int follows;

        FakeFollower(Pose pose) {
            this.pose = pose;
        }

        @Override public void follow(Path p) { busy = true; stopped = false; follows++; }
        @Override public boolean isBusy() { return busy; }
        @Override public Pose pose() { return pose; }
        @Override public Pose closestPose() { return pose; }
        @Override public void stop() { busy = false; stopped = true; }
        void finish() { busy = false; }
    }

    static final class FakeIntake implements IntakeStatus {
        int count;
        @Override public int ballCount() { return count; }
        @Override public int capacity() { return 4; }
    }

    private BallMap map;
    private FakeFollower follower;
    private FakeIntake intake;

    @Before
    public void setUp() {
        map = new BallMap();
        follower = new FakeFollower(new Pose(20, 20, 0));
        intake = new FakeIntake();
    }

    private CollectBrain brain(CollectRequest req) {
        return new CollectBrain(req, map, follower, intake, new FieldFrame(0),
                Collections.<ObstacleProvider>emptyList());
    }

    private static CollectRequest.Builder quiet() {
        return CollectRequest.builder(Alliance.RED, false).useHints(false).onlySources();
    }

    @Test
    public void startsFollowingAPlanToTheTarget() {
        MapSeeds.confirmed(map, BallClass.POLLEN, 40.0, 20.0);
        CollectBrain b = brain(quiet().build());
        b.start(100);
        assertEquals(1, follower.follows);
        assertEquals(1, b.plannedTargets().size());
        assertFalse(b.isDone());
    }

    @Test
    public void endsFullWhenTheIntakeFills() {
        MapSeeds.confirmed(map, BallClass.POLLEN, 40.0, 20.0);
        CollectBrain b = brain(quiet().build());
        b.start(100);
        intake.count = 4;
        b.update(200);
        assertTrue(b.isDone());
        assertEquals(CollectResult.FULL, b.result());
        assertTrue(follower.stopped);
    }

    @Test
    public void nothingToDoEndsWithNoTargets() {
        CollectBrain b = brain(quiet().build());
        b.start(100);
        assertEquals(CollectResult.NO_TARGETS, b.result());
    }

    @Test
    public void searchesASourceThenGivesUp() {
        CollectBrain b = brain(quiet().onlySources("LOADING_ZONE_RED").build());
        b.start(100);
        assertEquals(1, follower.follows);
        assertFalse(b.isDone());
        follower.finish();
        b.update(200);
        assertFalse(b.isDone());          // dwelling at the look pose
        b.update(700);
        assertEquals(CollectResult.NO_TARGETS, b.result());
    }

    @Test
    public void replansWhenAPlannedTargetDisappears() {
        MapSeeds.confirmed(map, BallClass.POLLEN, 40.0, 20.0, BallClass.POLLEN, 60.0, 30.0);
        CollectBrain b = brain(quiet().build());
        b.start(100);
        map.removePickup(new Pose(40 - 14.3, 20, 0));
        b.update(500);
        assertEquals(2, follower.follows);
        assertEquals(ReplanPolicy.Reason.TARGET_GONE, b.lastReason());
    }

    @Test
    public void skipsATargetAfterTwoMisses() {
        MapSeeds.confirmed(map, BallClass.POLLEN, 40.0, 20.0);
        CollectBrain b = brain(quiet().build());
        b.start(100);
        b.update(200);                    // intake mouth within reach: waiting for a pickup
        b.update(900);                    // grace over, no pickup: miss 1, re-plan
        assertFalse(b.isDone());
        assertTrue(b.skipped().isEmpty());
        b.update(1000);
        b.update(1700);                   // miss 2: skipped, nothing left
        assertEquals(1, b.skipped().size());
        assertEquals(CollectResult.NO_TARGETS, b.result());
    }

    @Test
    public void timesOut() {
        MapSeeds.confirmed(map, BallClass.POLLEN, 40.0, 20.0);
        CollectBrain b = brain(quiet().timeoutSec(1).build());
        b.start(100);
        b.update(1200);
        assertEquals(CollectResult.TIMEOUT, b.result());
    }

    @Test
    public void stopInterrupts() {
        MapSeeds.confirmed(map, BallClass.POLLEN, 40.0, 20.0);
        CollectBrain b = brain(quiet().build());
        b.start(100);
        b.stop();
        assertEquals(CollectResult.INTERRUPTED, b.result());
        assertTrue(follower.stopped);
    }

    @Test
    public void poseJumpForcesAReplan() {
        MapSeeds.confirmed(map, BallClass.POLLEN, 40.0, 20.0);
        CollectBrain b = brain(quiet().build());
        b.start(100);
        follower.pose = new Pose(20, 10, 0);
        b.update(150);
        assertEquals(ReplanPolicy.Reason.POSE_JUMP, b.lastReason());
        assertEquals(2, follower.follows);
    }
}
```

`skipsATargetAfterTwoMisses` relies on the robot at (20, 20, 0) having its intake mouth at (34.3, 20), which is 5.7 in from the ball at (40, 20), within `REACH_TOL_IN` = 6.

- [ ] **Step 2: Run it to make sure it fails**

Run: `./gradlew :TeamCode:testDebugUnitTest --tests 'org.firstinspires.ftc.teamcode.collect.CollectBrainTest'`
Expected: FAIL, "cannot find symbol: class CollectBrain".

- [ ] **Step 3: Implement**

`MAIN/collect/IntakeStatus.java`:

```java
package org.firstinspires.ftc.teamcode.collect;

/** What the collect command needs from the intake. The intake owner writes an adapter. */
public interface IntakeStatus {
    int ballCount();

    int capacity();

    default boolean isFull() {
        return ballCount() >= capacity();
    }
}
```

`MAIN/collect/PathFollower.java`:

```java
package org.firstinspires.ftc.teamcode.collect;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/** The slice of the Pedro follower the collect brain uses (faked in tests). */
public interface PathFollower {
    void follow(Path path);

    boolean isBusy();

    Pose pose();

    /** Closest point on the current path, or null when not following. */
    Pose closestPose();

    void stop();
}
```

`MAIN/collect/CollectResult.java`:

```java
package org.firstinspires.ftc.teamcode.collect;

public enum CollectResult {
    FULL,
    NO_TARGETS,
    TIMEOUT,
    INTERRUPTED
}
```

`MAIN/collect/CollectBrain.java`:

```java
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
    private enum State { IDLE, TARGETS, LOOK, DONE }

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
    private final Map<Integer, Integer> misses = new HashMap<Integer, Integer>();
    private final Set<String> visitedLooks = new HashSet<String>();

    private State state = State.IDLE;
    private CollectResult result;
    private List<MapBall> planned = Collections.emptyList();
    private List<Point> preview = Collections.emptyList();
    private long startMs;
    private long lookArrivedMs = -1;
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
        startMs = nowMs;
        lastCount = intake.ballCount();
        lastPose = follower.pose();
        policy.reset(nowMs);
        replan(nowMs);
    }

    public void update(long nowMs) {
        if (state == State.DONE) {
            return;
        }
        Pose pose = follower.pose();
        boolean jump = lastPose != null
                && Math.hypot(pose.x() - lastPose.x(), pose.y() - lastPose.y()) > CollectConfig.POSE_JUMP_IN;
        lastPose = pose;

        if (req.timeoutSec > 0 && nowMs - startMs >= (long) (req.timeoutSec * 1000)) {
            finish(CollectResult.TIMEOUT);
            return;
        }
        trackPickups(pose);
        if (req.endWhenFull && intake.isFull()) {
            finish(CollectResult.FULL);
            return;
        }
        if (state == State.TARGETS) {
            trackMisses(nowMs, pose);
        }
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
        in.pathDone = pathDone;
        in.following = !pathDone;
        in.pose = pose;
        in.targetGone = state == State.TARGETS && anyPlannedGone();
        in.unexpectedPickup = pendingUnexpected;
        in.missedTarget = pendingMissed;
        in.betterBall = state == State.LOOK ? hasTargets(pose) : betterBallAvailable(pose);
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
            if (p != null) {
                planned = new ArrayList<MapBall>(p.targets);
                preview = p.preview;
                follower.follow(p.path);
                state = State.TARGETS;
                return;
            }
        }
        planned = Collections.emptyList();
        while (true) {
            SearchPlanner.Look look = search.nextLook(pose, map.hints(), req, area, visitedLooks);
            if (look == null) {
                finish(CollectResult.NO_TARGETS);
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
        return planner.chooseTargets(pose, map.all(), req, area, cap, skip);
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
```

- [ ] **Step 4: Run the test to make sure it passes**

Run: `./gradlew :TeamCode:testDebugUnitTest --tests 'org.firstinspires.ftc.teamcode.collect.CollectBrainTest'`
Expected: PASS (9 tests).

- [ ] **Step 5: Run every test**

Run: `./gradlew :TeamCode:testDebugUnitTest`
Expected: PASS (all classes).

- [ ] **Step 6: Commit**

```bash
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/collect TeamCode/src/test/java/org/firstinspires/ftc/teamcode/collect/CollectBrainTest.java
git commit -m "Add collect brain state machine" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 13: NextFTC command, Pedro adapter, dashboard drawing, test OpMode

**Files:**
- Create: `MAIN/collect/PedroPathFollower.java`, `MAIN/collect/CollectCommand.java`, `MAIN/debug/FieldDrawer.java`, `MAIN/opmodes/RobotHooks.java`, `MAIN/opmodes/CountingIntakeStatus.java`, `MAIN/opmodes/CollectTest.java`

**Interfaces:**
- Consumes: everything above; Pedro `Follower` (`follow`, `isBusy`, `pose`, `following`, `closestPose`, `stop`, `update`, `manual(double, double, double)`); FTC Dashboard `Canvas`, `TelemetryPacket`, `FtcDashboard`
- Produces:
  - `PedroPathFollower(Follower)`
  - `CollectCommand(CameraSubsystem, BallTrackingSubsystem, PathFollower, IntakeStatus, CollectRequest, List<ObstacleProvider>)`, `asTeleopAssist(BooleanSupplier held, DoubleSupplier stick) -> Command`, `brain()`, `result()`
  - `FieldDrawer.draw(Canvas, Pose robot, BallMap, CollectBrain|null, FieldFrame)`
  - `RobotHooks.createFollower(HardwareMap) -> Follower` (the drivetrain team fills this in)

- [ ] **Step 1: Pedro adapter**

`MAIN/collect/PedroPathFollower.java`:

```java
package org.firstinspires.ftc.teamcode.collect;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/** PathFollower over a Pedro 3 Follower. Whoever owns the follower must call follower.update() every loop. */
public final class PedroPathFollower implements PathFollower {
    private final Follower follower;

    public PedroPathFollower(Follower follower) {
        this.follower = follower;
    }

    @Override public void follow(Path path) { follower.follow(path); }
    @Override public boolean isBusy() { return follower.isBusy(); }
    @Override public Pose pose() { return follower.pose(); }
    @Override public Pose closestPose() { return follower.following() ? follower.closestPose() : null; }
    @Override public void stop() { follower.stop(); }
}
```

- [ ] **Step 2: Collect command**

`MAIN/collect/CollectCommand.java`:

```java
package org.firstinspires.ftc.teamcode.collect;

import org.firstinspires.ftc.teamcode.perception.BallTrackingSubsystem;
import org.firstinspires.ftc.teamcode.perception.CameraMode;
import org.firstinspires.ftc.teamcode.perception.CameraSubsystem;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import dev.nextftc.core.commands.Command;

/**
 * NextFTC wrapper around CollectBrain. One instance = one run: build a new one each time.
 * Ends with result(): FULL, NO_TARGETS, TIMEOUT or INTERRUPTED. Chaining SCORE is the caller's job.
 */
public final class CollectCommand extends Command {
    private final CameraSubsystem camera;
    private final CollectBrain brain;

    public CollectCommand(CameraSubsystem camera, BallTrackingSubsystem tracking, PathFollower follower,
                          IntakeStatus intake, CollectRequest request, List<ObstacleProvider> providers) {
        this.camera = camera;
        this.brain = new CollectBrain(request, tracking.map(), follower, intake, FieldFrame.fromConfig(), providers);
        named("Collect");
    }

    @Override
    public void start() {
        camera.setMode(CameraMode.BALL);
        brain.start(System.currentTimeMillis());
    }

    @Override
    public void update() {
        brain.update(System.currentTimeMillis());
    }

    @Override
    public boolean isDone() {
        return brain.isDone();
    }

    @Override
    public void stop(boolean interrupted) {
        brain.stop();
    }

    /** TeleOp: runs while the button is held; releasing it or moving a stick cancels at once. */
    public Command asTeleopAssist(final BooleanSupplier held, final DoubleSupplier stick) {
        return until(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return !held.getAsBoolean() || Math.abs(stick.getAsDouble()) > CollectConfig.STICK_CANCEL;
            }
        });
    }

    public CollectBrain brain() { return brain; }
    public CollectResult result() { return brain.result(); }
}
```

- [ ] **Step 3: Dashboard drawing**

`MAIN/debug/FieldDrawer.java`:

```java
package org.firstinspires.ftc.teamcode.debug;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.pedropathing.math.Pose;
import com.skeletonarmy.marrow.zones.Point;

import org.firstinspires.ftc.teamcode.collect.CollectBrain;
import org.firstinspires.ftc.teamcode.collect.CollectConfig;
import org.firstinspires.ftc.teamcode.collect.FieldFrame;
import org.firstinspires.ftc.teamcode.collect.FieldRect;
import org.firstinspires.ftc.teamcode.perception.BallClass;
import org.firstinspires.ftc.teamcode.perception.BallMap;
import org.firstinspires.ftc.teamcode.perception.MapBall;

import java.util.List;

/** Draws pose, ball map, planned path and allowed area on the FTC Dashboard field (centre origin). */
public final class FieldDrawer {
    private FieldDrawer() {
    }

    public static void draw(Canvas c, Pose robot, BallMap map, CollectBrain brain, FieldFrame frame) {
        if (brain != null && brain.area().rect() != null) {
            FieldRect r = brain.area().rect();
            double[] a = dash(frame.toPedro(r.minX, r.minY));
            double[] b = dash(frame.toPedro(r.maxX, r.minY));
            double[] d = dash(frame.toPedro(r.maxX, r.maxY));
            double[] e = dash(frame.toPedro(r.minX, r.maxY));
            c.setStroke("#4CAF50").setStrokeWidth(1)
                    .strokePolygon(new double[]{a[0], b[0], d[0], e[0]}, new double[]{a[1], b[1], d[1], e[1]});
        }
        for (MapBall m : map.all()) {
            double[] p = dash(new double[]{m.x(), m.y()});
            String colour = colour(m.ballClass());
            c.setAlpha(m.isTarget() ? 1.0 : 0.4).setFill(colour).setStroke(colour);
            if (m.isConfirmed()) {
                c.fillCircle(p[0], p[1], m.ballClass().diameterIn / 2);
            } else {
                c.strokeCircle(p[0], p[1], m.ballClass().diameterIn / 2);
            }
        }
        c.setAlpha(1.0);
        if (brain != null && brain.preview().size() > 1) {
            List<Point> pts = brain.preview();
            double[] xs = new double[pts.size()];
            double[] ys = new double[pts.size()];
            for (int i = 0; i < pts.size(); i++) {
                double[] p = dash(new double[]{pts.get(i).getX(), pts.get(i).getY()});
                xs[i] = p[0];
                ys[i] = p[1];
            }
            c.setStroke("#9C27B0").setStrokeWidth(1).strokePolyline(xs, ys);
        }
        if (robot != null) {
            double[] p = dash(new double[]{robot.x(), robot.y()});
            double h = robot.heading() + Math.toRadians(CollectConfig.DASH_ROTATION_DEG);
            c.setStroke("#212121").setStrokeWidth(2).strokeCircle(p[0], p[1], 9)
                    .strokeLine(p[0], p[1], p[0] + 9 * Math.cos(h), p[1] + 9 * Math.sin(h));
        }
    }

    private static String colour(BallClass c) {
        switch (c) {
            case RED_NECTAR:
                return "#E53935";
            case BLUE_NECTAR:
                return "#1E88E5";
            default:
                return "#FBC02D";
        }
    }

    /** Pedro (corner origin) -> dashboard (centre origin), rotated by DASH_ROTATION_DEG. */
    private static double[] dash(double[] pedro) {
        double x = pedro[0] - 72;
        double y = pedro[1] - 72;
        double r = Math.toRadians(CollectConfig.DASH_ROTATION_DEG);
        return new double[]{x * Math.cos(r) - y * Math.sin(r), x * Math.sin(r) + y * Math.cos(r)};
    }
}
```

- [ ] **Step 4: Integration hooks and a manual intake counter**

`MAIN/opmodes/RobotHooks.java`:

```java
package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * The one place our code gets robot parts other people own. The drivetrain team replaces the
 * body with their Pedro 3 Follower (Mecanum + Pinpoint constants). Until then it fails loudly.
 */
public final class RobotHooks {
    private RobotHooks() {
    }

    public static Follower createFollower(HardwareMap hw) {
        throw new IllegalStateException(
                "RobotHooks.createFollower: plug in the drivetrain team's Pedro 3 Follower here");
    }
}
```

`MAIN/opmodes/CountingIntakeStatus.java`:

```java
package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.collect.IntakeStatus;

/** Bench stand-in for the intake: a driver presses a button when a ball goes in. */
public final class CountingIntakeStatus implements IntakeStatus {
    private final int capacity;
    private int count;

    public CountingIntakeStatus(int capacity) {
        this.capacity = capacity;
    }

    public void increment() { count = Math.min(capacity, count + 1); }
    public void reset() { count = 0; }
    @Override public int ballCount() { return count; }
    @Override public int capacity() { return capacity; }
}
```

- [ ] **Step 5: Collect test OpMode**

`MAIN/opmodes/CollectTest.java`:

```java
package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.collect.Alliance;
import org.firstinspires.ftc.teamcode.collect.CollectCommand;
import org.firstinspires.ftc.teamcode.collect.CollectConfig;
import org.firstinspires.ftc.teamcode.collect.CollectRequest;
import org.firstinspires.ftc.teamcode.collect.FieldFrame;
import org.firstinspires.ftc.teamcode.collect.ObstacleProvider;
import org.firstinspires.ftc.teamcode.collect.PedroPathFollower;
import org.firstinspires.ftc.teamcode.collect.ZonePreset;
import org.firstinspires.ftc.teamcode.debug.FieldDrawer;
import org.firstinspires.ftc.teamcode.perception.BallTrackingSubsystem;
import org.firstinspires.ftc.teamcode.perception.CameraSubsystem;

import java.util.Collections;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;

/**
 * Bench/practice OpMode. Init: D-pad up = alliance, D-pad right = zone preset.
 * Run: hold right bumper = collect (release or move a stick to cancel), X = "ball went in",
 * Y = reset count; otherwise sticks drive through the follower.
 */
@TeleOp(name = "Collect Test", group = "Perception")
public final class CollectTest extends NextFTCOpMode {
    private final CameraSubsystem camera = new CameraSubsystem();
    private final BallTrackingSubsystem tracking = new BallTrackingSubsystem(camera);
    private final CountingIntakeStatus intake = new CountingIntakeStatus(CollectConfig.INTAKE_CAPACITY);
    private Follower follower;
    private PedroPathFollower pathFollower;
    private Alliance alliance = Alliance.RED;
    private ZonePreset zone = ZonePreset.WHOLE_FIELD;
    private CollectCommand active;
    private Command activeAssist;
    private boolean lastUp;
    private boolean lastRight;
    private boolean lastX;
    private boolean lastY;

    public CollectTest() {
        addComponents(new SubsystemComponent(camera, tracking));
    }

    @Override
    public void onInit() {
        follower = RobotHooks.createFollower(hardwareMap);
        pathFollower = new PedroPathFollower(follower);
        camera.setPoseSource(new org.firstinspires.ftc.teamcode.perception.PoseSource() {
            @Override
            public com.pedropathing.math.Pose pose() {
                return follower.pose();
            }
        });
    }

    @Override
    public void onWaitForStart() {
        follower.update();
        if (gamepad1.dpad_up && !lastUp) {
            alliance = alliance == Alliance.RED ? Alliance.BLUE : Alliance.RED;
        }
        if (gamepad1.dpad_right && !lastRight) {
            zone = zone.next();
        }
        lastUp = gamepad1.dpad_up;
        lastRight = gamepad1.dpad_right;
        telemetry.addData("Alliance (dpad up)", alliance);
        telemetry.addData("Zone (dpad right)", zone);
        telemetry.update();
    }

    @Override
    public void onUpdate() {
        follower.update();
        if (gamepad1.x && !lastX) {
            intake.increment();
        }
        if (gamepad1.y && !lastY) {
            intake.reset();
        }
        lastX = gamepad1.x;
        lastY = gamepad1.y;

        if (activeAssist != null && !activeAssist.isScheduled()) {
            telemetry.addData("Last collect", active.result());
            activeAssist = null;
        }
        if (activeAssist == null && gamepad1.right_bumper && stick() < CollectConfig.STICK_CANCEL) {
            active = new CollectCommand(camera, tracking, pathFollower, intake,
                    CollectRequest.builder(alliance, false).zone(zone).build(),
                    Collections.<ObstacleProvider>emptyList());
            activeAssist = active.asTeleopAssist(new BooleanSupplier() {
                @Override
                public boolean getAsBoolean() {
                    return gamepad1.right_bumper;
                }
            }, new DoubleSupplier() {
                @Override
                public double getAsDouble() {
                    return stick();
                }
            });
            activeAssist.schedule();
        }
        if (activeAssist == null) {
            // Robot-centric manual drive; check the signs on the first run.
            follower.manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        }

        TelemetryPacket packet = new TelemetryPacket();
        FieldDrawer.draw(packet.fieldOverlay(), follower.pose(), tracking.map(),
                activeAssist != null ? active.brain() : null, FieldFrame.fromConfig());
        packet.put("balls", tracking.map().size());
        packet.put("targets", tracking.map().targets().size());
        packet.put("frame reject", String.valueOf(camera.lastReject()));
        packet.put("intake", intake.ballCount());
        if (activeAssist != null) {
            packet.put("collect state", active.brain().state());
            packet.put("last replan", String.valueOf(active.brain().lastReason()));
        }
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        telemetry.addData("Balls", tracking.map().size());
        telemetry.addData("Intake", intake.ballCount());
        telemetry.update();
    }

    private double stick() {
        return Math.max(Math.abs(gamepad1.left_stick_x),
                Math.max(Math.abs(gamepad1.left_stick_y), Math.abs(gamepad1.right_stick_x)));
    }
}
```

- [ ] **Step 6: Compile and run all tests**

Run: `./gradlew :TeamCode:compileDebugJavaWithJavac && ./gradlew :TeamCode:testDebugUnitTest`
Expected: `BUILD SUCCESSFUL` for both. If `follower.manual(double, double, double)` turns out to take arguments in another order, keep the call and fix the order: it is test-OpMode-only.

- [ ] **Step 7: Commit**

```bash
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/collect TeamCode/src/main/java/org/firstinspires/ftc/teamcode/debug TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes
git commit -m "Add collect command, dashboard drawing and Collect Test OpMode" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 14: Docs and spec sync

**Files:**
- Create: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/perception/README.md`
- Modify: `docs/superpowers/specs/2026-11-09-perception-collect-design.md`

- [ ] **Step 1: Write the usage README**

`MAIN/perception/README.md`:

````markdown
# Perception + collect

Ball detection on the Limelight 3A → field-frame ball map → Weaver/Pedro collect command.
Design: `docs/superpowers/specs/2026-11-09-perception-collect-design.md`.

## Wiring it into an OpMode

```java
CameraSubsystem camera = new CameraSubsystem();
BallTrackingSubsystem tracking = new BallTrackingSubsystem(camera);
addComponents(new SubsystemComponent(camera, tracking));          // constructor
camera.setPoseSource(() -> follower.pose());                      // onInit
// someone must call follower.update() every loop

// AUTO: collect, then the caller decides what's next
CollectCommand collect = new CollectCommand(camera, tracking, new PedroPathFollower(follower),
        intakeStatus, CollectRequest.builder(Alliance.RED, true).build(), Collections.emptyList());
collect.then(camera.modeCommand(CameraMode.GOAL) /* , your score command */).schedule();

// TeleOp: hold-to-collect
new CollectCommand(...).asTeleopAssist(() -> gamepad1.right_bumper, () -> stickMagnitude).schedule();
```

## Strategy knobs (CollectRequest)

classes, per-class weights, zone preset, ball sources (garden / loading zone / flowers),
hints on/off, max targets, timeout, end-when-full. AUTO always keeps to our half (manual G402).

## Tuning

All values are in `PerceptionConfig` and `CollectConfig`, live in FTC Dashboard (192.168.43.1:8080/dash).
Set `BALL_PITCH_DOWN_DEG`, `TILT_*_POS` and settle times from the prelim bench before trusting positions.

## Tests

`./gradlew :TeamCode:testDebugUnitTest` (needs `local.properties` with `sdk.dir`).

## Integration points owned by others

- `opmodes/RobotHooks.createFollower` — drivetrain team's Pedro 3 Follower.
- `collect/IntakeStatus` — intake team's adapter (`ballCount`, `capacity`).
- `PerceptionConfig.GOAL_PIPELINE` / `BALL_PIPELINES` — agree numbering with Vatsal.
````

- [ ] **Step 2: Sync the spec with the implemented details**

In `docs/superpowers/specs/2026-11-09-perception-collect-design.md`:
- In section 5 "Ball map update", replace `remove it` (in the sentence about the camera footprint) with `count a miss; remove it after MISS_FRAMES_TO_REMOVE (2) consecutive misses, so one dropped detector frame doesn't delete a ball`.
- In assumption 18's table, change the `sources` default cell to `loading zone (ours); gardens off (they score); flowers off until the intake confirms`.

- [ ] **Step 3: Full verification**

Run: `./gradlew :TeamCode:testDebugUnitTest && ./gradlew :TeamCode:compileDebugJavaWithJavac`
Expected: both `BUILD SUCCESSFUL`; the test-results XML files show 0 failures and 0 errors.

- [ ] **Step 4: Commit**

```bash
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/perception/README.md docs/superpowers/specs/2026-11-09-perception-collect-design.md
git commit -m "Document perception/collect usage and sync spec" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

## Not in this plan (next steps after it lands)

- Bench/prelim session to set `BALL_PITCH_DOWN_DEG`, tilt positions and settle times (port `LimelightPrelimBench` into TeamCode).
- On-field test of `Collect Test` once `RobotHooks.createFollower` exists.
- Model fine-tuning and data capture (separate plan).
````
