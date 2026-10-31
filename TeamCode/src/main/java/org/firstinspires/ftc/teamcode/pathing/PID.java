package org.firstinspires.ftc.teamcode.pathing;

/** Minimal PID. Gains are public so the follower can refresh them from the config every loop. */
public class PID {
    public double kP, kI, kD;
    public double integralLimit = 0.3;

    private double integral, lastError;
    private boolean hasLast;

    public double update(double error, double dt) {
        if (dt <= 0) dt = 1e-3;
        integral += error * dt;
        // Clamp the integral's contribution (not the raw sum) so changing kI live stays sane.
        if (kI > 0) {
            double max = integralLimit / kI;
            integral = Math.max(-max, Math.min(max, integral));
        }
        double derivative = hasLast ? (error - lastError) / dt : 0;
        lastError = error;
        hasLast = true;
        return kP * error + kI * integral + kD * derivative;
    }

    public void reset() {
        integral = 0;
        hasLast = false;
    }
}
