package org.firstinspires.ftc.teamcode.pathing;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/** Pinpoint wrapper reporting a {@link Pose} in inches / radians. */
public class PinpointLocalizer {
    private final GoBildaPinpointDriver pinpoint;
    private Pose pose = new Pose(0, 0, 0);
    private Pose velocity = new Pose(0, 0, 0);
    private long lastNanos;

    public PinpointLocalizer(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, PathingConfig.pinpointName);
        pinpoint.setOffsets(PathingConfig.xPodOffsetMm, PathingConfig.yPodOffsetMm, DistanceUnit.MM);
        pinpoint.setEncoderResolution(PathingConfig.swingarmPod
                ? GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD
                : GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                PathingConfig.xPodReversed ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                        : GoBildaPinpointDriver.EncoderDirection.FORWARD,
                PathingConfig.yPodReversed ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                        : GoBildaPinpointDriver.EncoderDirection.FORWARD);
        // Zeroes the position and recalibrates the IMU: keep the robot still during init.
        pinpoint.resetPosAndIMU();
    }

    public void update() {
        pinpoint.update();
        Pose2D p = pinpoint.getPosition();
        Pose next = new Pose(p.getX(DistanceUnit.INCH), p.getY(DistanceUnit.INCH), p.getHeading(AngleUnit.RADIANS));

        long now = System.nanoTime();
        if (lastNanos != 0) {
            double dt = (now - lastNanos) / 1e9;
            if (dt > 0) {
                velocity = new Pose((next.x - pose.x) / dt, (next.y - pose.y) / dt,
                        Pose.angleWrap(next.heading - pose.heading) / dt);
            }
        }
        lastNanos = now;
        pose = next;
    }

    public Pose getPose() {
        return pose;
    }

    /** Field-frame velocity in in/s and rad/s, from consecutive samples. */
    public Pose getVelocity() {
        return velocity;
    }

    public void setPose(Pose p) {
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, p.x, p.y, AngleUnit.RADIANS, p.heading));
        pose = p;
    }
}
