package org.firstinspires.ftc.teamcode.Experiments.Pinpoint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF.BCPath;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF.VectorField;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDFController;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.Vector;

@TeleOp
@Config
public class pid_point_test extends OpMode {
    Odometry odometry;
    WheelControl wheelControl;

    FtcDashboard dashboard;

    BCPath path;
    VectorField vf;
    ArrayList<Point> pathPoints;

    public static Point[][] follow_path;
    public static boolean uk = false;

    @Override
    public void init(){
        odometry = new Odometry(hardwareMap, telemetry, 7.875, 6.625, 0);
        wheelControl = new WheelControl(hardwareMap, odometry);
        dashboard = FtcDashboard.getInstance();

        Point[][] follow_path = {{
            new Point(12.1, 9),
            new Point(76.8, 5),
            new Point(97.3, 61.9),
            new Point(1.5, 100.6),
            new Point(109, 121.2),
        }};

        vf = new VectorField(wheelControl, odometry, uk);
        path = new BCPath(follow_path);
        vf.setPath(path, 90, true);

        pathPoints = path.get_path_points();

    }

    @Override
    public void loop(){
        odometry.update();

        if (gamepad1.triangle){
            odometry.reset_original_pos();
        }

        if (!vf.at_end(1)){
            vf.move();
        }

        telemetry.addData("X position", odometry.get_x(uk));
        telemetry.addData("Y position", odometry.get_y(uk));
        telemetry.addData("Direction", odometry.get_heading(uk));

        TelemetryPacket packet = new TelemetryPacket();

        packet.fieldOverlay() //draw robot
            .setTranslation(-72, 72)
            .setRotation(Math.toRadians(-90))
            .setStroke("blue")
            .strokeCircle(odometry.get_x(uk), odometry.get_y(uk), 15/2)
            .strokeLine(odometry.get_x(uk), odometry.get_y(uk),odometry.get_x(uk) + 15/2*Math.cos(odometry.get_heading(uk)), odometry.get_y(uk) + 15/2*Math.sin(odometry.get_heading(uk)));

        packet.fieldOverlay() //draw target path
            .setFill("red")
            .setStroke("orange")
            .fillCircle(7.875, 6.625, 2)
            .strokeLine(7.875, 6.625, pathPoints.get(0).x, pathPoints.get(0).y);

        for (int i=0; i<pathPoints.size()-1; i++) {
            packet.fieldOverlay()
                .strokeLine(pathPoints.get(i).x, pathPoints.get(i).y, pathPoints.get(i + 1).x, pathPoints.get(i + 1).y);
        }

        dashboard.sendTelemetryPacket(packet);
    }
}
