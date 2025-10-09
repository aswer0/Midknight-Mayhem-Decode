package org.firstinspires.ftc.teamcode.Experiments.Pinpoint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDFController;
import org.opencv.core.Point;

@TeleOp
@Config
public class pid_point_test extends OpMode {
    Odometry odometry;
    WheelControl wheelControl;

    public static double power = 1;
    public static double target_x = 40;
    public static double target_y = 40;
    public static double target_h = 90;
    public static double thresh = 0.2;

    FtcDashboard dashboard;

    @Override
    public void init(){
        odometry = new Odometry(hardwareMap, telemetry, 0, 0, 179);
        wheelControl = new WheelControl(hardwareMap, odometry);
        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop(){
        odometry.update();

        if (gamepad1.triangle){
            odometry.reset_original_pos();
        }

        telemetry.addData("X position", odometry.get_x());
        telemetry.addData("Y position", odometry.get_y());
        telemetry.addData("Direction", odometry.get_heading());
        telemetry.addData("Is at target", wheelControl.drive_to_point(
                new Point(target_x, target_y),
                target_h,
                power,
                thresh
        ));


    }
}
