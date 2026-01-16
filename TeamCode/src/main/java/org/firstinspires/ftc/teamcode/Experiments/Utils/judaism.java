package org.firstinspires.ftc.teamcode.Experiments.Utils;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "God is the greatest")
public class judaism extends OpMode {
    DcMotorEx FR;
    DcMotorEx FL;
    DcMotorEx BR;
    DcMotorEx BL;
    @Override
    public void init() {
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
    }

    @Override
    public void loop() {
        final double powerLevel = 0.5;

        FR.setPower(-gamepad1.right_stick_x+gamepad1.left_stick_y);
        FL.setPower(-gamepad1.right_stick_x+gamepad1.left_stick_y);
        BR.setPower(gamepad1.right_stick_x+gamepad1.left_stick_y);
        BL.setPower(gamepad1.right_stick_x+gamepad1.left_stick_y);
    }
}
