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
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Transfer.BeltTransfer;
import org.opencv.core.Point;

@TeleOp
@Config
public class B_DriveTest extends OpMode {
    WheelControl drive;
    public static Point start_point = new Point(8, 8);

    Odometry odo;
    Intake intake;
    Sensors sensors;
    BeltTransfer beltTransfer;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();

    FtcDashboard dashboard = FtcDashboard.getInstance();

    boolean useDriveCorrecton = true;
    boolean pidToPoint = false;
    public static Point shoot_point = new Point(60, 81);
    public static double target_shoot_heading = 135;
    int turnPower = 1;
    double drivePower = 1;
    boolean useKalmanOdo = false;

    @Override
    public void loop() {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);
        sensors = new Sensors(hardwareMap);
        intake = new Intake(hardwareMap, sensors);
        odo.update();

        if (currentGamepad1.options && !previousGamepad1.options) {
            //set heading to 0
            odo.set_heading(0);
        }
        if (currentGamepad1.share && !previousGamepad1.share) {
            //set heading to 0
            useDriveCorrecton = !useDriveCorrecton;
        }
        if (currentGamepad1.right_stick_button && !previousGamepad1.right_stick_button) {
            pidToPoint = true;
        }

        if (currentGamepad1.left_stick_button){
            drivePower = drivePower/(drivePower+1);
        }
        else{
            drivePower = 1;
        }

        if (!previousGamepad1.left_bumper && currentGamepad1.left_bumper) {
            intake.motorOn();
        } else if (!previousGamepad1.right_bumper && currentGamepad1.right_bumper) {
            intake.motorOff();
        }

        if (useDriveCorrecton && !pidToPoint){
            drive.correction_drive(gamepad1.left_stick_y, 1.2 * gamepad1.left_stick_x, -gamepad1.right_stick_x * turnPower, -Math.toRadians(-odo.get_heading(useKalmanOdo)), drivePower, false);
        }
        else{
            if (!pidToPoint){
                drive.drive(gamepad1.left_stick_y, 1.2 * gamepad1.left_stick_x, -gamepad1.right_stick_x * turnPower, -Math.toRadians(-odo.get_heading(useKalmanOdo)), drivePower);
            }
        }
        if (pidToPoint){
            drive.drive_to_point(shoot_point, target_shoot_heading, 1, 0.5, false);
        }


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
        odo = new Odometry(hardwareMap, telemetry, start_point.x, start_point.y, 0);
        drive = new WheelControl(hardwareMap, odo);

        odo.setOutputDebugInfo(false);

    }
}
