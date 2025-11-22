package org.firstinspires.ftc.teamcode.FinalCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Experiments.DrivetrainExperiments.Camera;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Outtake.Turret;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Outtake.Flywheel;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Transfer.ArmTransfer;
import org.opencv.core.Point;

@TeleOp
public class FinalTeleop extends OpMode {
    Odometry odo;
    WheelControl drive;
    Sensors sensors;
    Intake intake;
    Flywheel flywheel;
    ArmTransfer armTransfer;
    Turret turret;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();

    public static Point shoot_point = new Point(60, 81);
    public static Point target_shoot = new Point(11, 134);

    public static double target_shoot_heading = 135;

    double turnPower = 1;
    int drivePower = 1;
    boolean useKalmanOdo = false;
    boolean isTransferReady = true;
    boolean useDriveCorrecton = true;
    boolean pidToPoint = false;

    boolean stopFlywheel = false;
    Alliance alliance;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    enum Alliance{
        red,
        blue,
    }

    @Override
    public void init() {
        odo = new Odometry(hardwareMap, telemetry, 8, 8, 90);
        drive = new WheelControl(hardwareMap, odo);
        sensors = new Sensors(hardwareMap);
        intake = new Intake(hardwareMap, sensors);
        flywheel = new Flywheel(hardwareMap);
        alliance = Alliance.red;
        armTransfer = new ArmTransfer(hardwareMap, intake);
        turret = new Turret(hardwareMap, new Camera(hardwareMap), true);
    }

    @Override
    public void init_loop(){
        if (currentGamepad1.options && !previousGamepad1.options){
            if (alliance == Alliance.red){
                alliance = Alliance.blue;
            }
            else{
                alliance = Alliance.red;
            }
        }

        telemetry.addData("Alliance", alliance);
        telemetry.update();
    }

    @Override
    public void loop() {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);
        odo.update();
        flywheel.update();
        isTransferReady = armTransfer.update();

        if (currentGamepad1.options && !previousGamepad1.options) {
            //set heading to 0
            odo.set_heading(0);

            if (alliance == Alliance.red){
                odo.set_x(8);
                odo.set_y(8);
            }
            else if (alliance == Alliance.blue){
                odo.set_x(142-8);
                odo.set_y(8);
            }
        }
        if (currentGamepad1.share && !previousGamepad1.share) {
            //set heading to 0
            useDriveCorrecton = !useDriveCorrecton;
        }

        //drive
        if (useDriveCorrecton && !pidToPoint){
            drive.correction_drive(gamepad1.left_stick_y, 1.2 * gamepad1.left_stick_x, -gamepad1.right_stick_x * turnPower, -Math.toRadians(-odo.get_heading(useKalmanOdo)), drivePower, false);
        }
        else if (!pidToPoint){
            drive.drive(gamepad1.left_stick_y, 1.2 * gamepad1.left_stick_x, -gamepad1.right_stick_x * turnPower, -Math.toRadians(-odo.get_heading(useKalmanOdo)), drivePower);
        }
        else{
            drive.drive_to_point(new Point(77, 77), 45, 1, 0.5, false);
        }

        if (currentGamepad1.left_trigger > 0.3){
            pidToPoint = true;
        }
        else{
            pidToPoint = false;
        }

        //intake
        if (currentGamepad1.right_bumper) { //in
            intake.motorOn();
            flywheel.setTargetRPM(-670);
            stopFlywheel = true;
        } else if (currentGamepad1.right_trigger > 0.3) { //reverse
            intake.motorReverse();
            flywheel.setTargetRPM(-670);
            stopFlywheel = true;
        } else {
            if (gamepad1.left_bumper) { //transfer
                if (isTransferReady) {
                    armTransfer.transfer();
                }
            } else { //idle
                intake.motorOff();
            }
        }

        //flywheel
        //blue close
        //red far
        //green stop
        if (stopFlywheel && currentGamepad1.right_trigger <= 0.3){
            flywheel.stop();
            stopFlywheel = false;
        }
        else if (stopFlywheel && !currentGamepad1.right_bumper){
            flywheel.stop();
            stopFlywheel = false;
        }

        if (currentGamepad1.cross && !previousGamepad1.cross) {
            flywheel.stop();
        } else if (currentGamepad1.square && !previousGamepad1.square) {
            flywheel.shootClose();
        } else if (currentGamepad1.circle && !previousGamepad1.circle) {
            flywheel.shootFar();
        }

        if (flywheel.isReady()) {
            gamepad1.rumble(100);
        }
        turret.turret.setPower(-(gamepad1.dpad_left ? 0.6: 0) + (gamepad1.dpad_right ? 0.6: 0));
        telemetry.addData("power", drivePower);
        telemetry.update();

        Point pos = new Point(odo.get_x(useKalmanOdo), odo.get_y(useKalmanOdo));
        double dist = Math.sqrt((pos.x-target_shoot.x)*(pos.x-target_shoot.x) + (pos.y-target_shoot.y)*(pos.y-target_shoot.y));
        //flywheel.set_auto_rpm(dist);

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("distance", dist);
        packet.put("RPM", Flywheel.CLOSE_RPM);
        dashboard.sendTelemetryPacket(packet);
    }
}
