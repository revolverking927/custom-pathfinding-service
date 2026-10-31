package org.firstinspires.ftc.teamcode.pathing;

/** Field position in inches (origin at field center) and heading in radians (CCW positive). */
public final class Pose {
    public final double x, y, heading;

    public Pose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    /** Wraps an angle into (-PI, PI]. */
    public static double angleWrap(double radians) {
        while (radians > Math.PI) radians -= 2 * Math.PI;
        while (radians <= -Math.PI) radians += 2 * Math.PI;
        return radians;
    }

    public double distanceTo(Pose o) {
        return Math.hypot(o.x - x, o.y - y);
    }

    @Override
    public String toString() {
        return String.format("(%.1f, %.1f, %.0f deg)", x, y, Math.toDegrees(heading));
    }
}
