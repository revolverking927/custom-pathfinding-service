package org.firstinspires.ftc.teamcode.maincode;

//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Drivetrain_Mod {
    // This declares the four motors needed
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    IMU imu;
    Gamepad gamepad1;
    public Drivetrain_Mod(IMU imu, HardwareMap hardwareMap, Gamepad gamepad1){
        this.frontLeftDrive = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        this.frontRightDrive = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        this.backLeftDrive = hardwareMap.get(DcMotor.class, "leftBackMotor");
        this.backRightDrive = hardwareMap.get(DcMotor.class, "rightBackMotor");

        this.backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        this.frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        this.backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        this.frontRightDrive.setDirection(DcMotor.Direction.REVERSE);

        //Utilizing RUN_USING_ENCODER for accuracy
        this.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.imu = imu;
        this.gamepad1 = gamepad1;
    }

    public void start() {
        // If you press the A button, then you reset the Yaw to be zero from the way
        // the robot is currently pointing
        if (gamepad1.right_bumper) {
            imu.resetYaw();
        }
        // If you press the left bumper, you get a drive from the point of view of the robot
        // (much like driving an RC vehicle)
        if (gamepad1.left_bumper) {
            driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }
    }
    public void stop() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }

    // This routine drives the robot field relative
    private void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }

   // Main drive routine
    public void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = .70;
        double maxSpeed = .50;  // make this slower for outreaches

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
    }

}
