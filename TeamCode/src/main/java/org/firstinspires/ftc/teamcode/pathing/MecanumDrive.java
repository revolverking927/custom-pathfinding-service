package org.firstinspires.ftc.teamcode.pathing;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** Mecanum mixing and directions copied from maincode/Drivetrain_Mod, which is known to work. */
public class MecanumDrive {
    public final DcMotor frontLeft, frontRight, backLeft, backRight;

    public MecanumDrive(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotor.class, PathingConfig.frontLeftName);
        frontRight = hardwareMap.get(DcMotor.class, PathingConfig.frontRightName);
        backLeft = hardwareMap.get(DcMotor.class, PathingConfig.backLeftName);
        backRight = hardwareMap.get(DcMotor.class, PathingConfig.backRightName);

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    /**
     * Robot-relative drive. forward: +1 = forward, strafe: +1 = left, turn: +1 = clockwise
     * (matches Drivetrain_Mod's stick mapping, where positive rotation turns right).
     */
    public void drive(double forward, double strafe, double turn) {
        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;

        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))));
        frontLeft.setPower(fl / max);
        frontRight.setPower(fr / max);
        backLeft.setPower(bl / max);
        backRight.setPower(br / max);
    }

    public void stop() {
        drive(0, 0, 0);
    }
}
