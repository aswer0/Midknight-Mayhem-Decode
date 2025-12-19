package org.firstinspires.ftc.teamcode.Experiments.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Drivetrain.GVF.BCPath;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Drivetrain.GVF.VectorField;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Drivetrain.PIDdrive.Pathing;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Drivetrain.WheelControl;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.List;

@TeleOp
@Config
public class JaydenTest extends OpMode {
    public static Point start_point = new Point(25, 126);
    Odometry odometry;
    WheelControl wheelControl;

    FtcDashboard dashboard;
    Pathing pid_drive;

    /*
    (60,81)
    (57.84431880968166,64.08104407468775)
    (43.80173283594432,37.06720328549305)
    (21.49561723229877,35.37290537531175)

    * */

    public static ArrayList<Point> pathPoints = new ArrayList<>(List.of(
            new Point(60,81),
            new Point(43.80173283594432,37.06720328549305),
            new Point(21.49561723229877,35.37290537531175)
    ));

    public static boolean uk = false;

    @Override
    public void init(){
        odometry = new Odometry(hardwareMap, telemetry, start_point.x, start_point.y, 135);
        odometry.setOutputDebugInfo(true);
        wheelControl = new WheelControl(hardwareMap, odometry);
        dashboard = FtcDashboard.getInstance();
        pid_drive = new Pathing(odometry, wheelControl);
        pid_drive.set_path(pathPoints);

    }

    @Override
    public void loop(){
        odometry.update();

        if (gamepad1.triangle){
            odometry.reset_original_pos();
        }

        telemetry.addData("X position", odometry.get_x(uk));
        telemetry.addData("Y position", odometry.get_y(uk));
        telemetry.addData("Direction", odometry.get_heading(uk));
        telemetry.addData("Is at end", pid_drive.pointDriver(180, 0.7, 1, -1, uk, false));

        TelemetryPacket packet = new TelemetryPacket();

        packet.fieldOverlay() //draw robot
                .setTranslation(-72, 72)
                .setRotation(Math.toRadians(-90))
                .setStroke("blue")
                .strokeCircle(odometry.get_x(uk), odometry.get_y(uk), Math.sqrt(63))
                .strokeLine(odometry.get_x(uk), odometry.get_y(uk),odometry.get_x(uk) + Math.sqrt(63)*Math.cos(odometry.get_heading(uk)), odometry.get_y(uk) + Math.sqrt(63)*Math.sin(odometry.get_heading(uk)));

        packet.fieldOverlay() //draw target path
                .setFill("red")
                .setStroke("orange")
                .fillCircle(start_point.x, start_point.y, 2)
                .strokeLine(start_point.x, start_point.y, pathPoints.get(0).x, pathPoints.get(0).y);

        for (int i=0; i<pathPoints.size()-1; i++) {
            packet.fieldOverlay()
                    .strokeLine(pathPoints.get(i).x, pathPoints.get(i).y, pathPoints.get(i + 1).x, pathPoints.get(i + 1).y);
        }

        dashboard.sendTelemetryPacket(packet);
    }
}