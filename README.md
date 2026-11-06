# Custom Pathfinding Service

A small, self-owned pathing stack for an FTC mecanum robot with a goBILDA Pinpoint. It drives to a
pose or through a list of waypoints, has a driver-station tuner, and streams the robot's state to a
desktop viewer so you can watch it on a field grid. No Pedro, Dashboard or Panels; no extra Gradle
dependencies.

```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pathing/
  PathingConfig.java    hardware names, pod setup, tunable gains   <- the file you edit
  Follower.java         drive-to-pose / waypoint follower (call update() every loop)
  PathingTuner.java     the "Pathing Tuner" TeleOp
  PinpointLocalizer.java, MecanumDrive.java, PID.java, Pose.java, Grid.java
  FieldServer.java      serves the robot's state as JSON on port 8765
PathViewer/
  viewer.html           the desktop viewer (open it in any browser)
  simbot.js             a fake robot, so you can try the viewer without hardware
```

## Coordinates

- Origin is the field center. **+x is forward, +y is left**, heading is **counter-clockwise** positive.
- Everything is stored in inches and radians. `Grid.unit` only changes how you type and read numbers:
  `Grid.at(1, -2, 90)` means tiles when `Grid.unit = Grid.Unit.TILES`, inches otherwise.
  A tile is 24 in; the field is 144 in.
- `Grid.tile(col, row, headingDeg)` gives the center of a tile, `(0, 0)` being the -x/-y corner.

## 1. One-time setup

Open `PathingConfig.java` and fill in the hardware section. These are placeholders until you do:

| Field | What to set |
|---|---|
| `pinpointName` | the Pinpoint's name in your robot configuration |
| `frontLeftName`, `frontRightName`, `backLeftName`, `backRightName` | your drive motor names |
| `xPodOffsetMm`, `yPodOffsetMm` | pod offsets from the robot's tracking point in mm (X pod: left is +, Y pod: forward is +) |
| `swingarmPod` | `true` for the swingarm pod, `false` for the 4-bar pod |
| `xPodReversed`, `yPodReversed` | leave `false` for now; the tuner tells you if they need flipping |

Then build and install to the Robot Controller from Android Studio like any other FTC project.
The motor directions in `MecanumDrive` are copied from the known-good `Drivetrain_Mod`.

## 2. Open the viewer

The viewer is a single HTML file. The robot serves its state at `http://192.168.43.1:8765/state`
while the tuner is running, and the viewer polls it.

1. Join your PC to the Control Hub's Wi-Fi.
2. Double-click `PathViewer/viewer.html` (or open it in a browser). No server or install needed.
3. Leave **Robot URL** as `http://192.168.43.1:8765/state`, change it only if your hub uses another address.
4. Start the Pathing Tuner on the driver station. The dot in the viewer's header turns green once it connects.

The viewer shows the robot and its trail, the current target, the waypoints, pose, velocity, position
and heading error, motor power, and the current gains. Controls in the header: **Units** (match robot,
inches or tiles), **Robot size** (inches, for drawing the footprint), and **Clear trail**.

### Try it without the robot

`simbot.js` is a fake robot that runs the same follower maths against a simple mecanum model and
serves the same JSON. It needs [Node](https://nodejs.org) and nothing else.

```bash
node PathViewer/simbot.js
```

Open `viewer.html` and set **Robot URL** to `http://localhost:8765/state`. It drives one square and
then idles; restart it to run again. `SPEED=2 node PathViewer/simbot.js` runs faster, and
`HEADLESS=1 node PathViewer/simbot.js` skips the server and prints whether the square finished and
how far off it ended. Its speeds are guesses, so it shows that the logic converges, not how your robot
will behave.

If you use Claude Code or the Claude desktop app, `.claude/launch.json` defines a `path-viewer`
preview that serves the viewer on port 8123.

## 3. Tune the robot

Run the **Pathing Tuner** TeleOp (group "Tuning") on gamepad 1. Keep the robot still while the Pinpoint
initializes. Clear some floor space and have the stop button ready.

**Menu:** dpad up/down to pick a test, **A** to start it, **B** to stop and return to the menu.
**BACK** toggles inches / tiles.

Do the tests in order. The first three catch wrong signs, which cause most "bad PID" symptoms.

| # | Test | What to do | What you want |
|---|---|---|---|
| 1 | Localizer check | Push the robot by hand. **X** zeroes the pose. | Forward makes x go up, left makes y go up, turning CCW makes heading go up. If a sign is wrong, flip `xPodReversed` / `yPodReversed`. |
| 2 | Motor check | Hold **X** (front left), **Y** (front right), **A** (back left), **RB** (back right), with the robot on blocks. | Every wheel spins as if the robot were driving forward. |
| 3 | Drive direction check | **A** runs forward, strafe left, then turn CCW, about 0.6 s each. | All three say PASS. A FAIL message says whether to flip a pod or fix a motor direction. |
| 4 | Heading tune | **A** turns to 90° and back to 0°. | See below. |
| 5 | Translation tune | **A** drives 24 in forward, back, 24 in left, back. | See below. |
| 6 | Square test | **A** drives a 24 in square, turning at each corner. | Ends within tolerance of where it started. |

### Tuning the gains

On tests 4 to 6, **A** starts the next move, **X** zeroes the pose, **B** is the emergency stop,
and these tweak the gains live:

- **Dpad left/right** picks a parameter, **dpad up/down** changes it.
- **Right bumper** makes the step 10x bigger, **left bumper** 10x smaller.
- **Y** saves the values to a file on the Robot Controller and prints them as lines you can paste into `PathingConfig.java`.

Saved values are loaded automatically on the next run and override the defaults in the file. Once you
are happy, paste the values into `PathingConfig.java` so they live in your code.

Suggested order and what to watch in the viewer's error and power readouts:

1. **Heading first** (`headP`, `headD`, then `headI` only if it stalls short). Raise `headP` until it
   turns briskly, then add `headD` if it overshoots and rings. Back off if it oscillates.
2. **Translation** (`transP`, `transD`). Raise `transP` until it reaches the target quickly; add
   `transD` if it overshoots or coasts past. Leave `transI` at 0 unless it stops short and stays there.
3. **`kStatic`** is the minimum power added while still outside tolerance. Raise it if the robot
   stalls just short of the target, lower it if it creeps or jitters near the end.
4. **`maxPower`** caps speed. Lower it while tuning, raise it once it is stable.
5. **Tolerances** (`posTolIn`, `headTolDeg`, `settleSec`) decide when a waypoint counts as reached.
   Tighter is more accurate but takes longer to settle.
6. Finish with the **square test**. Edges will bow slightly because the robot turns while it drives.

## 4. Use it in your own OpMode

```java
Follower follower = new Follower(hardwareMap);
follower.localizer.setPose(Grid.at(-2, -2, 0));     // starting pose
follower.followWaypoints(Arrays.asList(
        Grid.at(-1, -2, 0),
        Grid.at(-1, 0, 90),
        Grid.tile(2, 3, 180)));

while (opModeIsActive() && follower.isBusy()) {
    follower.update();       // updates the localizer and drives the motors
}
```

Call `follower.update()` every loop, even when idle, if you want a current pose. `goTo(pose)` drives to
one pose, `cancel()` stops. To feed the viewer from your own OpMode, start a `FieldServer` on
`PathingConfig.VIEWER_PORT`, call `server.publish(follower.stateJson("MY MODE"))` each loop, and
`server.stop()` in a `finally` block so the port is free next run.

## Troubleshooting

- **Viewer stays "Connecting" / "Not connected":** the PC must be on the Control Hub's Wi-Fi, and the
  Pathing Tuner (or your OpMode with a `FieldServer`) must be running. The viewer only has data while
  the OpMode is active.
- **Robot drives the wrong way in test 3:** read the FAIL message; it distinguishes a pod sign problem
  from a motor direction problem.
- **Viewer axes look off:** x is up on screen and y is left, matching the robot's coordinates.
- **Building on a low-memory PC:** `./gradlew -Dorg.gradle.jvmargs="-Xmx768m" :TeamCode:compileDebugJavaWithJavac -q`
  compiles without the full IDE, and needs `JAVA_HOME` and `ANDROID_HOME` set.
