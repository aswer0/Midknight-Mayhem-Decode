package org.firstinspires.ftc.teamcode.Experiments.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.FinalCode.FinalTeleop;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Outtake.Turret;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Transfer.BeltTransfer;
import org.opencv.core.Point;

@Disabled
@TeleOp
@Config
public class B_DriveTest extends OpMode {
    WheelControl drive;
    public static Point start_point = new Point(8, 8);

    Odometry odo;
    Intake intake;
    Sensors sensors;
    Turret turret;
    BeltTransfer beltTransfer;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();

    FtcDashboard dashboard = FtcDashboard.getInstance();
    public static PIDFCoefficients turretCoefficients = new PIDFCoefficients(0.02, 0.003, 0.00025,0.2);

    int turnPower = 1;
    double drivePower = 1;
    boolean useKalmanOdo = false;

    @Override
    public void loop() {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);
        odo.update();
        turret.update();

        if (currentGamepad1.options && !previousGamepad1.options) {
            //set heading to 0
            odo.set_heading(0);
        }

        if (!previousGamepad1.left_bumper && currentGamepad1.left_bumper) {
            intake.motorOn();
        } else if (!previousGamepad1.right_bumper && currentGamepad1.right_bumper) {
            intake.motorOff();
        }

        drive.correction_drive(-gamepad1.left_stick_y, 1.2 * -gamepad1.left_stick_x, -gamepad1.right_stick_x * turnPower, -Math.toRadians(-odo.get_heading(useKalmanOdo)), drivePower, false);

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
        sensors = new Sensors(hardwareMap);
        intake = new Intake(hardwareMap, sensors);
        turret = new Turret(hardwareMap, null, odo, FinalTeleop.Alliance.blue, false, turretCoefficients);
        turret.autoAiming = true;

        odo.setOutputDebugInfo(false);

    }
}
