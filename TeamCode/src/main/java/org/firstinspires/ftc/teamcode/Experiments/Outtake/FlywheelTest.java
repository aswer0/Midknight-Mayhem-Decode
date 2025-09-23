package org.firstinspires.ftc.teamcode.Experiments.Outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDFController;

@TeleOp
@Config
public class FlywheelTest extends OpMode {
    double target = 0;
    double RPM = 0;
    double error = 0;
    public static double kp = 0;
    public static double ki = 0;
    public static double kd = 0;

    DcMotorEx flywheel;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
//
//    PIDFController flywheelController = new PIDFController(kp, ki, kd, 0);
//
    @Override
    public void init() {
//        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
//        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
//
    @Override
    public void loop() {
//        previousGamepad1.copy(currentGamepad1);
//        currentGamepad1.copy(gamepad1);
//
//        if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
//            target -= 50;
//        } else if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
//            target += 50;
//        }
//
//        RPM = (flywheel.getVelocity()) / 28 * 60;
//        error =  target - RPM;
//
//        flywheel.setPower(flywheelController.update(error));
//
//        telemetry.addData("RPM", RPM);
    }
}
