package org.firstinspires.ftc.teamcode.pathing;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;

/**
 * Drives to a pose, or through a list of waypoints. Call {@link #update()} every loop; it also
 * refreshes the localizer, so call it even when idle if you want a current pose.
 */
public class Follower {
    public final PinpointLocalizer localizer;
    public final MecanumDrive drive;

    private final PID transPid = new PID();
    private final PID headPid = new PID();
    private final List<Pose> waypoints = new ArrayList<>();
    private int index = -1;
    private boolean busy;
    private long settleStartNanos;
    private long lastNanos;

    private double posError, headError;
    private double outForward, outStrafe, outTurn;

    public Follower(HardwareMap hardwareMap) {
        localizer = new PinpointLocalizer(hardwareMap);
        drive = new MecanumDrive(hardwareMap);
    }

    public void goTo(Pose target) {
        followWaypoints(Arrays.asList(target));
    }

    public void followWaypoints(List<Pose> path) {
        waypoints.clear();
        waypoints.addAll(path);
        index = waypoints.isEmpty() ? -1 : 0;
        busy = index >= 0;
        settleStartNanos = 0;
        lastNanos = 0;
        transPid.reset();
        headPid.reset();
    }

    public void cancel() {
        busy = false;
        index = -1;
        waypoints.clear();
        outForward = outStrafe = outTurn = 0;
        drive.stop();
    }

    public boolean isBusy() {
        return busy;
    }

    public Pose getTarget() {
        return (index >= 0 && index < waypoints.size()) ? waypoints.get(index) : null;
    }

    public void update() {
        localizer.update();
        if (!busy) return;

        long now = System.nanoTime();
        double dt = lastNanos == 0 ? 0.02 : (now - lastNanos) / 1e9;
        lastNanos = now;

        Pose pose = localizer.getPose();
        Pose target = waypoints.get(index);

        double dx = target.x - pose.x;
        double dy = target.y - pose.y;
        posError = Math.hypot(dx, dy);
        headError = Pose.angleWrap(target.heading - pose.heading);

        boolean inPos = posError <= PathingConfig.posTolIn;
        boolean inHead = Math.abs(headError) <= Math.toRadians(PathingConfig.headTolDeg);
        boolean last = index == waypoints.size() - 1;

        if (inPos && inHead) {
            if (!last) {
                advance();
                return;
            }
            if (settleStartNanos == 0) settleStartNanos = now;
            if ((now - settleStartNanos) / 1e9 >= PathingConfig.settleSec) {
                busy = false;
                outForward = outStrafe = outTurn = 0;
                drive.stop();
                return;
            }
        } else {
            settleStartNanos = 0;
        }

        transPid.kP = PathingConfig.transP;
        transPid.kI = PathingConfig.transI;
        transPid.kD = PathingConfig.transD;
        headPid.kP = PathingConfig.headP;
        headPid.kI = PathingConfig.headI;
        headPid.kD = PathingConfig.headD;

        // Translation: one PID on the distance, direction taken from the error vector, so a
        // diagonal move keeps its heading instead of one axis finishing first.
        double power = 0;
        double fwdDir = 0, strDir = 0;
        if (!inPos) {
            power = transPid.update(posError, dt);
            power = Math.min(PathingConfig.maxPower, Math.max(power, 0) + PathingConfig.kStatic);
            double c = Math.cos(pose.heading), s = Math.sin(pose.heading);
            fwdDir = (dx * c + dy * s) / posError;
            strDir = (-dx * s + dy * c) / posError;
        } else {
            transPid.reset();
        }

        double turnCcw = 0;
        if (!inHead) {
            turnCcw = headPid.update(headError, dt);
            turnCcw += Math.signum(headError) * PathingConfig.kStatic;
            turnCcw = Math.max(-PathingConfig.maxPower, Math.min(PathingConfig.maxPower, turnCcw));
        } else {
            headPid.reset();
        }

        outForward = fwdDir * power;
        outStrafe = strDir * power;
        outTurn = -turnCcw; // MecanumDrive turn is clockwise-positive, heading is CCW-positive
        drive.drive(outForward, outStrafe, outTurn);
    }

    private void advance() {
        index++;
        settleStartNanos = 0;
        transPid.reset();
        headPid.reset();
    }

    public double getPosError() {
        return posError;
    }

    /** Heading error in radians. */
    public double getHeadError() {
        return headError;
    }

    public double getOutForward() { return outForward; }
    public double getOutStrafe() { return outStrafe; }
    public double getOutTurn() { return outTurn; }

    /** State for the desktop viewer, as JSON. Positions are inches / radians. */
    public String stateJson(String mode) {
        Pose p = localizer.getPose();
        Pose v = localizer.getVelocity();
        Pose t = getTarget();
        StringBuilder sb = new StringBuilder(512);
        sb.append("{\"time\":").append(System.currentTimeMillis());
        sb.append(",\"mode\":\"").append(mode).append('"');
        sb.append(",\"unit\":\"").append(Grid.unit.name()).append('"');
        sb.append(",\"fieldSize\":").append(Grid.FIELD_SIZE_IN);
        sb.append(",\"pose\":").append(poseJson(p));
        sb.append(",\"velocity\":").append(poseJson(v));
        sb.append(",\"target\":").append(t == null ? "null" : poseJson(t));
        sb.append(",\"waypoints\":[");
        for (int i = 0; i < waypoints.size(); i++) {
            if (i > 0) sb.append(',');
            sb.append(poseJson(waypoints.get(i)));
        }
        sb.append("],\"waypointIndex\":").append(index);
        sb.append(",\"busy\":").append(busy);
        sb.append(String.format(Locale.US,
                ",\"posError\":%.3f,\"headError\":%.4f,\"power\":{\"f\":%.3f,\"s\":%.3f,\"t\":%.3f}",
                posError, headError, outForward, outStrafe, outTurn));
        sb.append(",\"gains\":{");
        for (int i = 0; i < PathingConfig.TUNABLE.length; i++) {
            if (i > 0) sb.append(',');
            sb.append('"').append(PathingConfig.TUNABLE[i]).append("\":")
                    .append(PathingConfig.get(PathingConfig.TUNABLE[i]));
        }
        sb.append("}}");
        return sb.toString();
    }

    private static String poseJson(Pose p) {
        return String.format(Locale.US, "{\"x\":%.3f,\"y\":%.3f,\"h\":%.4f}", p.x, p.y, p.heading);
    }
}
