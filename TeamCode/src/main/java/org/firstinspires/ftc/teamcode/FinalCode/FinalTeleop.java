package org.firstinspires.ftc.teamcode.FinalCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Experiments.DrivetrainExperiments.Camera;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.LED;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Outtake.Turret;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Outtake.Flywheel;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Transfer.ArmTransfer;
import org.opencv.core.Point;

@TeleOp
@Config
public class FinalTeleop extends OpMode {
    Odometry odo;
    WheelControl drive;
    Sensors sensors;
    Intake intake;
    Flywheel flywheel;
    ArmTransfer armTransfer;
    Turret turret;
    LED led;

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
    boolean useAutoRPM = false;
    boolean stopFlywheel = false;
    boolean doorClose = true;

    int x_sign;
    int y_sign;

    public static Alliance alliance = Alliance.red;
    public static double startX = 8;
    public static double startY = 8;
    public static boolean outputDebugInfo = true;
    public static double startHeading = 0.0;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    public enum Alliance{
        red,
        blue,
    }

    @Override
    public void init() {
        odo = new Odometry(hardwareMap, telemetry, startX, startY, startHeading);
        drive = new WheelControl(hardwareMap, odo);
        sensors = new Sensors(hardwareMap);
        intake = new Intake(hardwareMap, sensors);
        flywheel = new Flywheel(hardwareMap);
        armTransfer = new ArmTransfer(hardwareMap, intake);
        turret = new Turret(hardwareMap, new Camera(hardwareMap), odo, alliance, false);
        led = new LED(hardwareMap, sensors, flywheel);
        turret.CURRENT_VOLTAGE = hardwareMap.voltageSensor.iterator().next().getVoltage();
    }

    @Override
    public void init_loop(){
        telemetry.addData("Alliance", alliance);
        telemetry.update();
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        if (currentGamepad1.triangle && !previousGamepad1.triangle){
            if (alliance == Alliance.blue){
                alliance = Alliance.red;

            }
            else{
                alliance = Alliance.blue;
            }
        }

        if (alliance == Alliance.red){
            x_sign = 1;
            y_sign = 1;
            target_shoot = new Point(142-11, 134);
            odo.set_x(CloseAutoRedSide.park_point.x); //used to be 122
            odo.set_y(CloseAutoRedSide.park_point.y); //81
        }
        else if (alliance == Alliance.blue){
            x_sign = -1;
            y_sign = -1;
            target_shoot = new Point(11, 134);
            odo.set_x(startX); //used to be 20
            odo.set_y(startY); //81
        }
        turret.alliance = alliance;
    }

    @Override
    public void loop() {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);
        odo.update();
        flywheel.update();
        isTransferReady = armTransfer.update();
        led.speedCheck();
        turret.update();
        if (currentGamepad1.options && !previousGamepad1.options) {
            if (alliance == Alliance.red){
                odo.set_heading(0);
                odo.set_x(122);
                odo.set_y(81);
            }
            else if (alliance == Alliance.blue){
                odo.set_heading(180);
                odo.set_x(20);
                odo.set_y(81);
            }
        }
        if (currentGamepad1.share && !previousGamepad1.share) {
            //set heading to 0
            useDriveCorrecton = !useDriveCorrecton;
        }

        //drive
        if (useDriveCorrecton && !pidToPoint){
            drive.correction_drive(x_sign*gamepad1.left_stick_y, y_sign*1.2 * gamepad1.left_stick_x, -gamepad1.right_stick_x * turnPower, -Math.toRadians(-odo.get_heading(useKalmanOdo)), drivePower, false);
        }
        else if (!pidToPoint){
            drive.drive(x_sign*gamepad1.left_stick_y, y_sign*1.2 * gamepad1.left_stick_x, -gamepad1.right_stick_x * turnPower, -Math.toRadians(-odo.get_heading(useKalmanOdo)), drivePower);
        }
        else{
            drive.drive_to_point(new Point(127, 14), -45, 1, 0.5, false);
        }

        pidToPoint = currentGamepad1.left_trigger > 0.3;

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
                intake.motorOn();
                doorClose = false;
            } else { //idle
                intake.motorOff();
                doorClose = true;
            }
        }
        if (doorClose){
            intake.doorUp();
        }
        else{
            intake.doorDown();
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
            useAutoRPM = false;
            turret.autoAiming = false;
            turret.setAngle(0);
        } else if (currentGamepad1.square && !previousGamepad1.square) {
            flywheel.shootClose();
        } else if (currentGamepad1.circle && !previousGamepad1.circle) {
            flywheel.shootFar();
            turret.autoAiming = true;
        } else if (currentGamepad1.triangle && !previousGamepad1.triangle) {
            useAutoRPM = true;
            turret.autoAiming = true;
        }


        if (flywheel.isReady()) {
            gamepad1.rumble(100);
        }
        //if(!turret.autoAiming) turret.turret.setPower(-(gamepad1.dpad_left ? 0.6: 0) + (gamepad1.dpad_right ? 0.6: 0));
        telemetry.addData("power", drivePower);
        telemetry.addData("transferStage", armTransfer.transferStage);
        telemetry.addData("Correction Drive?", useDriveCorrecton);
        telemetry.update();

        double future_x = odo.get_x_predicted(false, false);
        double future_y = odo.get_y_predicted(false, false);

        double dist = Math.sqrt((future_x-target_shoot.x)*(future_x-target_shoot.x) + (future_y-target_shoot.y)*(future_y-target_shoot.y));

        if (useAutoRPM){
            flywheel.shootAutoDist();
            flywheel.set_auto_rpm(dist);
        }

        if (outputDebugInfo) {
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("distance", dist);
            packet.put("x", odo.get_x(false));
            packet.put("y", odo.get_y(false));
            packet.put("Auto RPM", Flywheel.AUTO_RPM);
            packet.put("RPM", flywheel.getCurrentRPM());
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
