package org.firstinspires.ftc.teamcode.maincode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Outtake_Mod {

    DcMotor pollenShooter;
    DcMotor nectarShooter;
    DcMotor pollenTransfer;
    Gamepad gamepad1;
    public Outtake_Mod(HardwareMap hardwareMap, Gamepad gamepad1) {
        this.pollenShooter = hardwareMap.get(DcMotor.class, "pollenShooter");
        this.nectarShooter = hardwareMap.get(DcMotor.class,"nectarShooter");
        this.pollenTransfer = hardwareMap.get(DcMotor.class,"pollenTransfer");

        this.gamepad1 = gamepad1;

    }

    public void start() {
        if (gamepad1.dpad_up) {
            pollenShooter.setPower(.9);
        } else if (gamepad1.dpad_down) {
            pollenShooter.setPower(-.9);
        } else {
            pollenShooter.setPower(0);
        }
        if (gamepad1.y) {
            nectarShooter.setPower(1);
        } else if (gamepad1.a) {
            nectarShooter.setPower(-1);
        } else {
            nectarShooter.setPower(0);
        }
        if (gamepad1.x) {
            pollenTransfer.setPower(-1);
        } else if (gamepad1.b) {
            pollenTransfer.setPower(1);
        } else {
            pollenTransfer.setPower(0);
        }
    }
}

