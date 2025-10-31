package org.firstinspires.ftc.teamcode.Experiments;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Drivetrain.WheelControl;
import org.opencv.core.Point;

@TeleOp
@Config
public class B_PIDtoPointTest extends OpMode {
    public static Point start_point = new Point(8, 8);
    WheelControl drive;
    Odometry odo;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();

    public static Point target = new Point(30, 30);
    public static double power = 1;
    public static double dist_thresh = 1;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void init() {
        odo = new Odometry(hardwareMap, telemetry, start_point.x, start_point.y, 0);
        drive = new WheelControl(hardwareMap, odo);

        odo.setOutputDebugInfo(true);
    }

    @Override
    public void loop() {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);
        odo.update();

        boolean at_point = drive.drive_to_point(target, 90, power, dist_thresh, false);

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("At point", at_point);
        packet.put("X", odo.get_x(false));
        packet.put("Y", odo.get_y(false));
        packet.put("heading", odo.get_heading(false));
        dashboard.sendTelemetryPacket(packet);

    }
}