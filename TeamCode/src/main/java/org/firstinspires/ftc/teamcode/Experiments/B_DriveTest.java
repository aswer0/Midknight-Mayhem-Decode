package org.firstinspires.ftc.teamcode.Experiments;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake.Intake;

@TeleOp
@Config
public class B_DriveTest extends OpMode {
    double drivePower = 1;
    WheelControl drive;

    public static double x_sign = -1;
    public static double y_sign = -1;
    public static double h_sign = -1;
    public static double r_sign = -1;
    public static double oh_sign = -1;

    Odometry odo;
    Intake intake;
    Sensors sensors;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();

    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void loop() {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);
        sensors = new Sensors(hardwareMap);
        intake = new Intake(hardwareMap, sensors);
        odo.update();

        if (!previousGamepad1.left_bumper && currentGamepad1.left_bumper) {
            intake.motorOn();
        } else if (!previousGamepad1.right_bumper && currentGamepad1.right_bumper) {
            intake.motorOff();
        }

        double y = gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double r = gamepad1.right_stick_x;

        drive.drive(y_sign*y, x_sign*1.2*x, r_sign*r, h_sign*Math.toRadians(oh_sign*odo.get_heading(false)), drivePower);

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("X position", odo.get_x(false));
        packet.put("Y position", odo.get_y(false));
        packet.put("Heading position", odo.get_heading(false));
        dashboard.sendTelemetryPacket(packet);

        telemetry.addData("X position", odo.get_x(false));
        telemetry.addData("Y position", odo.get_y(false));
        telemetry.addData("Heading position", odo.get_heading(false));
        telemetry.update();
    }
    @Override
    public void init() {
        odo = new Odometry(hardwareMap, telemetry, 7.875, 6.625, 0);
        drive = new WheelControl(hardwareMap, odo);

        odo.setOutputDebugInfo(false);

    }
}
