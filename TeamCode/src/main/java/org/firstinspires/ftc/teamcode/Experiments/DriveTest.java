package org.firstinspires.ftc.teamcode.Experiments;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Drivetrain.Odometry;

@TeleOp
public class DriveTest extends OpMode {
    double drivePower = 1;
    boolean fieldOriented = false;
    double botHeading = 0;

    DcMotorEx BL;
    DcMotorEx BR;
    DcMotorEx FL;
    DcMotorEx FR;

    Odometry odo;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();

    @Override
    public void init() {
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");

        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);

        odo = new Odometry(hardwareMap, telemetry, 0, 0, 0);
    }

    @Override
    public void loop() {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);
        odo.update();

        if (!previousGamepad1.left_bumper && currentGamepad1.left_bumper) {
            drivePower -= 0.1;
        } else if (!previousGamepad1.right_bumper && currentGamepad1.right_bumper) {
            drivePower += 0.1;
        }

        if (!previousGamepad1.options && currentGamepad1.options) {
            botHeading = 0;
            fieldOriented = !fieldOriented;
        }

        if (fieldOriented) {
            botHeading = Math.toRadians(odo.get_heading(false));
        }

        //debug motor ports
        if (gamepad1.square) FL.setPower(1);
        if (gamepad1.triangle) FR.setPower(1);
        if (gamepad1.circle) BR.setPower(1);
        if (gamepad1.cross) BL.setPower(1);

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX *= 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        BL.setPower(backLeftPower);
        BR.setPower(backRightPower);
        FL.setPower(frontLeftPower);
        FR.setPower(frontRightPower);

        telemetry.addData("Drive power", drivePower);
        telemetry.addData("Field oriented", fieldOriented);
    }
}
