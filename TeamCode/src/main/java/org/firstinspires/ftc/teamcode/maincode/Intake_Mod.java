package org.firstinspires.ftc.teamcode.maincode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake_Mod {
    DcMotor intakeMotor;
    Gamepad gamepad1;
    public Intake_Mod(HardwareMap hardwareMap, Gamepad gamepad1) {
        this.intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        this.gamepad1 = gamepad1;
    }

    public void start() {
        if (gamepad1.right_trigger > 0.1) {
            intakeMotor.setPower(1.0);
        } else if (gamepad1.left_trigger > 0.1) {
            intakeMotor.setPower(-1.0);
        } else {
            intakeMotor.setPower(0.0);
        }
    }
}
