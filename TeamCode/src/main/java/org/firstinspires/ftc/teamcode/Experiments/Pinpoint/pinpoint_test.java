package org.firstinspires.ftc.teamcode.Experiments.Pinpoint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;

@TeleOp
public class pinpoint_test extends OpMode {
    Servo turret;

    Odometry odometry;
    WheelControl wheelControl;

    double power = 0.9;

    @Override
    public void init(){
        turret = hardwareMap.get(Servo.class, "turret");

        odometry = new Odometry(hardwareMap, telemetry, 7.875, 6.625, 0);
        wheelControl = new WheelControl(hardwareMap, odometry);
    }

    @Override
    public void loop(){
        odometry.update();

        double y = gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        if (gamepad1.triangle){
            odometry.reset_original_pos();
        }

        wheelControl.correction_drive(-y, -x, rx*0.7, -Math.toRadians(odometry.get_heading()), power);

//        double turretAngle = odometry.get_turret_heading();
//        double servoPos = (turretAngle + 90) / 180.0;
//        servoPos = Math.max(0, Math.min(1, servoPos));
//
//        turret.setPosition(servoPos);

        telemetry.addData("X position", odometry.get_x());
        telemetry.addData("Y position", odometry.get_y());
        telemetry.addData("Direction", odometry.get_heading());
        telemetry.addData("Turret Direction", odometry.get_turret_heading());
        telemetry.addData("Turret Servo Pos", odometry.get_turret_heading()/300);

    }
}
