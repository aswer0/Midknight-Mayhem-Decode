package org.firstinspires.ftc.teamcode.FinalCode;

import static org.firstinspires.ftc.teamcode.FinalCode.Constants.shootWaitTime;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Experiments.DrivetrainExperiments.Camera;
import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.LED;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Outtake.Hood;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Outtake.Model;
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
    Hood hood;
    /** Transfer opens first, then shoots */
    ElapsedTime timer = new ElapsedTime();

    ElapsedTime doorTimer = new ElapsedTime();
    ElapsedTime transferTimer = new ElapsedTime();

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    public static Point shoot_point = new Point(60, 81);
    public static Point blue_gate = new Point(9.5, 59.4);
    public static Point red_gate = new Point(130.2, 59.4);
    public static boolean use_gain_schedule = false;
    public static double hood_angle = 30;
    public static Point target_shoot = new Point(11, 134);
    public static PIDFCoefficients turretCoefficients = new PIDFCoefficients(0.02, 0.003, 0.00025,0.2);

    double turnPower = 1;
    int drivePower = 1;
    boolean useKalmanOdo = false;
    boolean useDriveCorrecton = true;
    boolean pidToGate = false;
    boolean turretReady = false;
    boolean shootFar = false;
    boolean inCloseZone = false;
    boolean lastInCloseZone = false;
    boolean hasBall = false;

    int x_sign;
    int y_sign;

    public static Alliance alliance = Alliance.red;
    public static double startX = 8;
    public static double startY = 8;
    public static boolean outputDebugInfo = false;
    public static double startHeading = 0.0;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    public static int idleRpm = 2467;

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
        turret = new Turret(hardwareMap, null, odo, alliance, false, turretCoefficients);
        led = new LED(hardwareMap, sensors, flywheel);
        hood = new Hood(hardwareMap);

        turret.CURRENT_VOLTAGE = hardwareMap.voltageSensor.iterator().next().getVoltage();
        flywheel.CURRENT_VOLTAGE = hardwareMap.voltageSensor.iterator().next().getVoltage();
        flywheel.use_gained_schedule = use_gain_schedule;

        turret.autoAiming = true;
        flywheel.set_tele_coeffs();
    }

    @Override
    public void start() {
        intake.doorOpen();
        doorTimer.reset();
        transferTimer.reset();
        timer.reset();
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
            target_shoot = new Point(136, 136);
            odo.set_x(startX); //used to be 122
            odo.set_y(startY); //81
        }
        else if (alliance == Alliance.blue){
            x_sign = -1;
            y_sign = -1;
            target_shoot = new Point(18, 136);
            odo.set_x(startX); //used to be 20
            odo.set_y(startY); //81
        }
        turret.alliance = alliance;
    }

    @Override
    public void loop() {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);
        odo.update();
        flywheel.update();
        led.speedCheck();
        turretReady = turret.update();

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

        pidToGate = currentGamepad1.left_stick_button;

        //drive
        if (useDriveCorrecton && !pidToGate){
            drive.correction_drive(x_sign*gamepad1.left_stick_y, y_sign*1.2 * gamepad1.left_stick_x, -gamepad1.right_stick_x * turnPower, -Math.toRadians(-odo.get_heading(useKalmanOdo)), drivePower, false);
        }
        else if (!pidToGate){
            drive.drive(x_sign*gamepad1.left_stick_y, y_sign*1.2 * gamepad1.left_stick_x, -gamepad1.right_stick_x * turnPower, -Math.toRadians(-odo.get_heading(useKalmanOdo)), drivePower);
        }
        else{
            if (alliance == Alliance.blue) {
                drive.drive_to_point(blue_gate, 155, 0.67, 0.5, false);
            } else {
                drive.drive_to_point(red_gate, 25, 0.67, 0.5, false);
            }
        }

        //intake
        if (currentGamepad1.right_bumper) { //in
            if (sensors.hasMidBall() || sensors.hasBackBall()) hasBall = true;
            // If we have 3 balls, auto stop intake
            if(sensors.hasAllBalls()) {
                intake.motorOff();
                gamepad1.rumble(500);
            } else {
                intake.motorOn();
                intake.doorClose();
            }
            doorTimer.reset();
        } else if (currentGamepad1.right_trigger > 0.3) { //reverse
            intake.motorReverse();
        } else {
            if (doorTimer.milliseconds() > 400) intake.doorOpen();
            if (gamepad1.left_bumper) { //transfer
                hasBall = false;
                if (transferTimer.milliseconds() > shootWaitTime) {
                    doorTimer.reset();
                    intake.doorClose();
                    intake.motorOn();
                } else {
                    intake.doorOpen();
                    intake.motorOn();
                }
            } else if (currentGamepad1.left_trigger > 0.3) { //slow transfer
                hasBall = false;
                intake.doorOpen();
                intake.intervalTransfer(timer.milliseconds(), 210, 280);
            }else { //idle
                intake.motorOff();
                transferTimer.reset();
            }
        }

        if (currentGamepad1.cross) {
            shootFar = false;
        } else if (currentGamepad1.circle) {
            shootFar = true;
        }

        double future_x = odo.get_x_predicted(false, false);
        double future_y = odo.get_y_predicted(false, false);

        double dist = Math.sqrt((future_x-target_shoot.x)*(future_x-target_shoot.x) + (future_y-target_shoot.y)*(future_y-target_shoot.y));

        if (odo.inCloseZone(24) || hasBall) {
            if (dist > 115 && shootFar) {
                flywheel.shootAutoDist();
                flywheel.set_auto_far_rpm(dist);
            } else {
                dist = Math.min(dist, 115);
                flywheel.shootAutoDist();
                flywheel.set_auto_rpm(dist);
            }
        } else {
            flywheel.setTargetRPM(idleRpm);
        }

        if ((currentGamepad1.dpad_right && !previousGamepad1.dpad_right) || (currentGamepad2.dpad_right && !previousGamepad2.dpad_right)) {
                odo.set_heading(odo.get_heading(false)+2);
        } else if ((currentGamepad1.dpad_left && !previousGamepad1.dpad_left) || (currentGamepad2.dpad_left && !previousGamepad2.dpad_left)) {
                odo.set_heading(odo.get_heading(false)-2);
        }

        lastInCloseZone = inCloseZone;

        hood_angle = Model.auto_hood;
        hood.set_angle(hood_angle);
        Model.rpm_curr = flywheel.getCurrentRPM();
        Model.update_values(dist);

        telemetry.update();
        if (outputDebugInfo) {
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("distance", dist);
            packet.put("x", odo.get_x(false));
            packet.put("y", odo.get_y(false));
            packet.put("Auto RPM", Flywheel.AUTO_RPM);
            packet.put("RPM", flywheel.getCurrentRPM());
            packet.put("Model RPM", Model.auto_rpm);
            packet.put("Model Hood Angle", Model.auto_hood);
            packet.put("Model RPM current", Model.rpm_curr);
            packet.put("Turret Heading Reggin", turret.getAngle());
            packet.put("Odo heading", odo.get_heading(false));
            packet.put("Has ball", hasBall);
            packet.put("in close zone", odo.inCloseZone(24));
            packet.put("MODEL/AUTO_HOOD", Model.auto_hood);

            packet.put("AAAAAAAAAAAAAAAA flywheel left current", flywheel.flywheel_left.getCurrent(CurrentUnit.AMPS));
            packet.put("AAAAAAAAAAAAAAA intake 1 current", intake.intakeMotor.getCurrent(CurrentUnit.AMPS));
            packet.put("AAAAAAAAAAAAAAA intake 2 current", intake.intakeMotorTwo.getCurrent(CurrentUnit.AMPS));

            dashboard.sendTelemetryPacket(packet);
        }
    }
}
