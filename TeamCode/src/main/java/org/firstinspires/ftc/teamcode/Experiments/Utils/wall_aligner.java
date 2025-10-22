package org.firstinspires.ftc.teamcode.Experiments.Utils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp
@Config
public class wall_aligner extends OpMode {
    IMU imu;
    DcMotorEx BL;
    DcMotorEx BR;
    DcMotorEx FL;
    DcMotorEx FR;

    public static double y = 0;
    public static double x = 0;
    public static double rx = 0;

    @Override
    public void init() {
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");

        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {
        if (gamepad1.square) FL.setPower(1);
        if (gamepad1.triangle) FR.setPower(1);
        if (gamepad1.circle) BR.setPower(1);
        if (gamepad1.cross) BL.setPower(1);

        double botHeading = 0;

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        BL.setPower(backLeftPower * 0.2);
        BR.setPower(backRightPower * 0.2);
        FL.setPower(frontLeftPower * 0.2);
        FR.setPower(frontRightPower * 0.2);
    }
}
