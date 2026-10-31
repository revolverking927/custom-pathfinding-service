package org.firstinspires.ftc.teamcode.maincode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Intake", group = "Examples")
public class Intake extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.right_trigger > 0.1) {
                intakeMotor.setPower(1.0);
            } else if (gamepad1.left_trigger > 0.1) {
                intakeMotor.setPower(-1.0);
            } else {
                intakeMotor.setPower(0.0);
            }
        }
    }
}
