package org.firstinspires.ftc.teamcode.pathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.IOException;
import java.util.Arrays;
import java.util.List;

/**
 * Menu-driven tuner. Dpad up/down + A to pick a test, B to stop and go back.
 * Run the first three checks before tuning; they catch wrong signs, which cause most "bad PID"
 * symptoms. Open PathViewer/viewer.html on a PC joined to the Control Hub Wi-Fi to watch the robot.
 */
@TeleOp(name = "Pathing Tuner", group = "Tuning")
public class PathingTuner extends LinearOpMode {
    private enum Mode {MENU, LOCALIZER, MOTORS, DIRECTION, HEADING, TRANSLATION, SQUARE}

    private static final Mode[] TESTS = {Mode.LOCALIZER, Mode.MOTORS, Mode.DIRECTION,
            Mode.HEADING, Mode.TRANSLATION, Mode.SQUARE};
    private static final String[] TEST_NAMES = {
            "1. Localizer check (push the robot)",
            "2. Motor check",
            "3. Drive direction check",
            "4. Heading tune",
            "5. Translation tune",
            "6. Square test"};

    private static final List<Pose> HEADING_SEQ = Arrays.asList(
            new Pose(0, 0, Math.toRadians(90)), new Pose(0, 0, 0));
    private static final List<Pose> TRANSLATION_SEQ = Arrays.asList(
            new Pose(24, 0, 0), new Pose(0, 0, 0), new Pose(0, 24, 0), new Pose(0, 0, 0));
    private static final List<Pose> SQUARE_PATH = Arrays.asList(
            new Pose(24, 0, Math.toRadians(90)), new Pose(24, 24, Math.toRadians(180)),
            new Pose(0, 24, Math.toRadians(-90)), new Pose(0, 0, 0));

    private static final double CHECK_POWER = 0.3;
    private static final double DIR_DRIVE_SEC = 0.6, DIR_REST_SEC = 0.4;

    private Follower follower;
    private Mode mode = Mode.MENU;
    private int menuIndex, tweakIndex, seqIndex;
    private double stepMul = 1;

    // Drive direction check state
    private int dirStep = -1;
    private long dirStepStart;
    private Pose dirStartPose;
    private final String[] dirResult = new String[3];

    private final boolean[] last = new boolean[16];

    @Override
    public void runOpMode() {
        PathingConfig.load();
        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("Initializing Pinpoint (keep the robot still)...");
        telemetry.update();
        follower = new Follower(hardwareMap);

        FieldServer server = new FieldServer();
        String viewerStatus;
        try {
            server.start(PathingConfig.VIEWER_PORT);
            viewerStatus = "Viewer: http://192.168.43.1:" + PathingConfig.VIEWER_PORT + "/state";
        } catch (IOException e) {
            viewerStatus = "Viewer unavailable: " + e.getMessage();
        }
        telemetry.addLine("Ready. " + viewerStatus);
        telemetry.update();

        waitForStart();
        try {
            while (opModeIsActive()) {
                follower.update();
                if (rise(0, gamepad1.back)) Grid.toggleUnit();

                switch (mode) {
                    case MENU: menu(); break;
                    case LOCALIZER: localizerTest(); break;
                    case MOTORS: motorTest(); break;
                    case DIRECTION: directionTest(); break;
                    case HEADING: tuneTest(HEADING_SEQ, null); break;
                    case TRANSLATION: tuneTest(TRANSLATION_SEQ, null); break;
                    case SQUARE: tuneTest(null, SQUARE_PATH); break;
                }

                server.publish(follower.stateJson(mode.name()));
                telemetry.addData("Viewer", viewerStatus);
                telemetry.update();
            }
        } finally {
            follower.cancel();
            server.stop();
        }
    }

    // ---- Modes ----

    private void menu() {
        if (rise(1, gamepad1.dpad_down)) menuIndex = (menuIndex + 1) % TESTS.length;
        if (rise(2, gamepad1.dpad_up)) menuIndex = (menuIndex + TESTS.length - 1) % TESTS.length;
        if (rise(3, gamepad1.a)) {
            follower.cancel();
            seqIndex = 0;
            dirStep = -1;
            mode = TESTS[menuIndex];
            return;
        }
        telemetry.addLine("PATHING TUNER  (dpad + A to select, BACK = inches/tiles)");
        for (int i = 0; i < TESTS.length; i++) {
            telemetry.addLine((i == menuIndex ? "> " : "  ") + TEST_NAMES[i]);
        }
        showPose();
    }

    private void localizerTest() {
        if (backOut()) return;
        if (rise(4, gamepad1.x)) follower.localizer.setPose(new Pose(0, 0, 0));
        telemetry.addLine("Push the robot FORWARD: x should go UP.");
        telemetry.addLine("Push it LEFT: y should go UP.");
        telemetry.addLine("Turn it COUNTER-CLOCKWISE: heading should go UP.");
        telemetry.addLine("X = zero pose, B = back");
        telemetry.addLine("Wrong sign? Flip xPodReversed / yPodReversed in PathingConfig.");
        showPose();
    }

    private void motorTest() {
        if (backOut()) return;
        MecanumDrive d = follower.drive;
        d.frontLeft.setPower(gamepad1.x ? CHECK_POWER : 0);
        d.frontRight.setPower(gamepad1.y ? CHECK_POWER : 0);
        d.backLeft.setPower(gamepad1.a ? CHECK_POWER : 0);
        d.backRight.setPower(gamepad1.right_bumper ? CHECK_POWER : 0);
        telemetry.addLine("Hold X = front left, Y = front right, A = back left, RB = back right");
        telemetry.addLine("Every wheel should spin so the robot would drive FORWARD. B = back");
        telemetry.addData("FL / FR / BL / BR", "%s / %s / %s / %s", PathingConfig.frontLeftName,
                PathingConfig.frontRightName, PathingConfig.backLeftName, PathingConfig.backRightName);
    }

    private void directionTest() {
        if (backOut()) return;
        if (dirStep < 0) {
            telemetry.addLine("Clear ~1 ft around the robot. A = run (forward, strafe left, turn CCW). B = back");
            if (rise(3, gamepad1.a)) startDirStep(0);
        } else if (dirStep < 3) {
            double t = (System.nanoTime() - dirStepStart) / 1e9;
            if (t < DIR_DRIVE_SEC) {
                follower.drive.drive(dirStep == 0 ? CHECK_POWER : 0,
                        dirStep == 1 ? CHECK_POWER : 0,
                        dirStep == 2 ? -CHECK_POWER : 0); // turn is clockwise-positive: negative = CCW
            } else {
                follower.drive.stop();
                if (t >= DIR_DRIVE_SEC + DIR_REST_SEC) {
                    evaluateDirStep();
                    if (dirStep < 2) startDirStep(dirStep + 1);
                    else dirStep = 3;
                }
            }
            telemetry.addLine("Running step " + (dirStep + 1) + "/3...");
        } else {
            telemetry.addLine("A = run again. B = back");
            if (rise(3, gamepad1.a)) startDirStep(0);
        }
        String[] names = {"Forward ", "Strafe L", "Turn CCW"};
        for (int i = 0; i < 3; i++) {
            if (dirResult[i] != null) telemetry.addLine(names[i] + ": " + dirResult[i]);
        }
        showPose();
    }

    private void startDirStep(int step) {
        dirStep = step;
        dirStepStart = System.nanoTime();
        dirStartPose = follower.localizer.getPose();
        if (step == 0) Arrays.fill(dirResult, null);
    }

    private void evaluateDirStep() {
        Pose p = follower.localizer.getPose(), s = dirStartPose;
        double dx = p.x - s.x, dy = p.y - s.y;
        double c = Math.cos(s.heading), sn = Math.sin(s.heading);
        double fwd = dx * c + dy * sn;
        double str = -dx * sn + dy * c;
        double turn = Pose.angleWrap(p.heading - s.heading);
        switch (dirStep) {
            case 0:
                dirResult[0] = fwd > 0.5 ? String.format("PASS (%.1f in)", fwd)
                        : "FAIL: pinpoint saw " + String.format("%.1f in", fwd)
                        + ". Robot went forward? flip xPodReversed. Went backward? fix motor directions.";
                break;
            case 1:
                dirResult[1] = str > 0.5 ? String.format("PASS (%.1f in)", str)
                        : "FAIL: pinpoint saw " + String.format("%.1f in", str)
                        + ". Robot went left? flip yPodReversed. Went right? fix motor directions.";
                break;
            default:
                dirResult[2] = turn > 0.05 ? String.format("PASS (%.0f deg)", Math.toDegrees(turn))
                        : "FAIL: pinpoint saw " + String.format("%.0f deg", Math.toDegrees(turn))
                        + ". Robot turned CCW? check Pinpoint IMU / pod offsets. Turned CW? fix motor directions.";
        }
    }

    /** Heading / translation / square tests: A starts the next move, X zeroes the pose. */
    private void tuneTest(List<Pose> sequence, List<Pose> path) {
        if (backOut()) return;
        if (rise(4, gamepad1.x)) {
            follower.cancel();
            follower.localizer.setPose(new Pose(0, 0, 0));
        }
        if (rise(6, gamepad1.y)) {
            PathingConfig.save();
            telemetry.log().add("Saved. Paste into PathingConfig:\n" + PathingConfig.dump());
        }
        if (rise(3, gamepad1.a) && !follower.isBusy()) {
            if (path != null) {
                follower.followWaypoints(path);
            } else {
                follower.goTo(sequence.get(seqIndex));
                seqIndex = (seqIndex + 1) % sequence.size();
            }
        }
        tweak();

        telemetry.addLine(follower.isBusy() ? "RUNNING (B = emergency stop)" : "Idle: A = start next move");
        telemetry.addLine("X = zero pose, Y = save, BACK = inches/tiles");
        Pose t = follower.getTarget();
        if (t != null) {
            telemetry.addData("Target", "(%.2f, %.2f) %.0f deg", Grid.toUnit(t.x), Grid.toUnit(t.y), Math.toDegrees(t.heading));
        }
        telemetry.addData("Error", "%.2f in, %.1f deg", follower.getPosError(), Math.toDegrees(follower.getHeadError()));
        telemetry.addData("Power f/s/t", "%.2f / %.2f / %.2f", follower.getOutForward(), follower.getOutStrafe(), follower.getOutTurn());
        showPose();
    }

    // ---- Helpers ----

    private void tweak() {
        int n = PathingConfig.TUNABLE.length;
        if (rise(7, gamepad1.dpad_right)) tweakIndex = (tweakIndex + 1) % n;
        if (rise(8, gamepad1.dpad_left)) tweakIndex = (tweakIndex + n - 1) % n;
        if (rise(9, gamepad1.right_bumper)) stepMul = Math.min(stepMul * 10, 1000);
        if (rise(10, gamepad1.left_bumper)) stepMul = Math.max(stepMul / 10, 0.001);
        String name = PathingConfig.TUNABLE[tweakIndex];
        double delta = PathingConfig.STEP[tweakIndex] * stepMul;
        if (rise(11, gamepad1.dpad_up)) PathingConfig.set(name, PathingConfig.get(name) + delta);
        if (rise(12, gamepad1.dpad_down)) PathingConfig.set(name, Math.max(0, PathingConfig.get(name) - delta));

        telemetry.addLine("Tweak: dpad L/R = param, U/D = change, bumpers = step x10 / ÷10");
        for (int i = 0; i < n; i++) {
            telemetry.addLine((i == tweakIndex ? "> " : "  ") + PathingConfig.TUNABLE[i] + " = "
                    + String.format("%.4f", PathingConfig.get(PathingConfig.TUNABLE[i])));
        }
        telemetry.addData("Step multiplier", stepMul);
    }

    private void showPose() {
        Pose p = follower.localizer.getPose();
        telemetry.addData("Units", Grid.unit);
        telemetry.addData("Pose", "x %.2f  y %.2f  heading %.1f deg",
                Grid.toUnit(p.x), Grid.toUnit(p.y), Math.toDegrees(p.heading));
    }

    /** B: stop everything and return to the menu. */
    private boolean backOut() {
        if (rise(13, gamepad1.b)) {
            follower.cancel();
            dirStep = -1;
            mode = Mode.MENU;
            return true;
        }
        return false;
    }

    /** True on the first loop a button is pressed. */
    private boolean rise(int slot, boolean now) {
        boolean r = now && !last[slot];
        last[slot] = now;
        return r;
    }
}
