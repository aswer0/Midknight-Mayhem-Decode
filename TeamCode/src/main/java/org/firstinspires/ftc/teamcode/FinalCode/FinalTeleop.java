package org.firstinspires.ftc.teamcode.FinalCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experiments.DrivetrainExperiments.Camera;
import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDFCoefficients;
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
    /** Transfer opens first, then shoots */
    ElapsedTime transferDelay = new ElapsedTime();
    ElapsedTime timer = new ElapsedTime();

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    public static Point shoot_point = new Point(60, 81);
    public static Point blue_gate = new Point(9.5, 59.4);
    public static Point red_gate = new Point(127.7, 59.4);
    public static boolean use_gain_schedule = false;
    public static Point target_shoot = new Point(11, 134);
    public static PIDFCoefficients turretCoefficients = new PIDFCoefficients(0.02, 0.003, 0.00025,0.2);

    public static double target_shoot_heading = 135;

    double turnPower = 1;
    int drivePower = 1;
    boolean useKalmanOdo = false;
    boolean isTransferReady = true;
    boolean useDriveCorrecton = true;
    public boolean shouldStopIntake = false;
    boolean pidToGate = false;
    boolean pidToPoint = false;
    boolean useAutoRPM = false;
    boolean stopFlywheel = false;
    boolean turretReady = false;
    boolean hasBackBall = false;
    boolean hasMidBall = false;
    boolean shootFar = false;
    boolean triangle = false;

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
        turret.CURRENT_VOLTAGE = hardwareMap.voltageSensor.iterator().next().getVoltage();
        flywheel.CURRENT_VOLTAGE = hardwareMap.voltageSensor.iterator().next().getVoltage();
        flywheel.use_gained_schedule = use_gain_schedule;

        turret.autoAiming = false;
        flywheel.set_tele_coeffs();
    }

    @Override
    public void start() {
        intake.doorOpen();
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
            //odo.set_heading(0);
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
//        isTransferReady = armTransfer.update();
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

        //pidToPoint = currentGamepad1.left_trigger > 0.3;
        pidToGate = currentGamepad1.left_stick_button;

        //intake
        if (currentGamepad1.right_bumper) { //in
//            flywheel.setTargetRPM(-670);
            if (!odo.inCloseZone(24)) {
                flywheel.setTargetRPM(idleRpm);
                useAutoRPM = false;
                if (flywheel.getCurrentRPM() < idleRpm + 150) {
                    turret.autoAiming = false;
                    turret.setAngle(0);
                }
            }
            //stopFlywheel = true;
            // If we have 3 balls, auto stop intake
            hasBackBall = sensors.getBackColor() == 1;
            hasMidBall = sensors.getMidColor() == 1;
            if(shouldStopIntake || hasBackBall && hasMidBall && intake.intakeCurrentThreshold(6.7) == 1) {
                intake.motorOff();
                gamepad1.rumble(500);
                shouldStopIntake = true;
            } else {
                intake.motorOn();
                intake.doorClose();
            }
        } else if (currentGamepad1.right_trigger > 0.3) { //reverse
            intake.motorReverse();
//            flywheel.setTargetRPM(-670);
            stopFlywheel = true;
        } else {
            if (gamepad1.left_bumper) { //transfer
//                if (isTransferReady) {
//                    armTransfer.transfer();
                    intake.doorOpen();
//                }
                if(intake.doorOpen || transferDelay.seconds() > 0.5) {
                    intake.motorOn();
                } else intake.motorOff();
            } else if (gamepad1.left_trigger > 0.3) { //slow transfer
                intake.doorOpen();
                if(intake.doorOpen || transferDelay.seconds() > 0.5) {
                    intake.intervalTransfer(timer.milliseconds(), 200, 400);
                } else intake.motorOff();
            }else { //idle
                intake.motorOff();
                timer.reset();
            }
        }
        if(!gamepad1.left_bumper)
            transferDelay.reset();
        if(!currentGamepad1.right_bumper) shouldStopIntake = false;
        //flywheel
        //blue close
        //red far
        //green stop
        if (stopFlywheel && currentGamepad1.right_trigger <= 0.3){
            flywheel.setTargetRPM(idleRpm);
            stopFlywheel = false;
        }
        else if (stopFlywheel && !currentGamepad1.right_bumper){
            flywheel.setTargetRPM(idleRpm);
            stopFlywheel = false;
//            turret.autoAiming = true;
        }

        if (!triangle) {
            if (hasBackBall || hasMidBall) {
                useAutoRPM = true;
                if (odo.inCloseZone(24) || shootFar) {
                    turret.autoAiming = true;
                } else {
                    turret.autoAiming = false;
                }
            } else if (!odo.inCloseZone(24)) {
                if (!shootFar) {
                    useAutoRPM = false;
                    turret.autoAiming = false;
                }
            }
        }

        if (currentGamepad1.cross && !previousGamepad1.cross) { //stop shooter circle
            flywheel.setTargetRPM(idleRpm);
            useAutoRPM = false;
            turret.autoAiming = false;
            shootFar = false;
            triangle = false;
            turret.setAngle(0);
        } else if (currentGamepad1.square && !previousGamepad1.square) { //manual shoot close triangle
            intake.doorOpen();
            flywheel.shootClose();
            turret.autoAiming = false;
            turret.setAngle(0);
            shootFar = false;
        } else if (currentGamepad1.circle && !previousGamepad1.circle) { // shoot far cross
            intake.doorOpen();
            flywheel.shootFar();
            turret.autoAiming = true;
            shootFar = true;
        } else if (currentGamepad1.triangle && !previousGamepad1.triangle) { //auto shoot close square
//            useAutoRPM = true;
            useAutoRPM = false; //NEW
            flywheel.shootClose(); //NEW
            turret.autoAiming = true;
            shootFar = false;
            triangle = true;
        }

        if ((currentGamepad1.dpad_right && !previousGamepad1.dpad_right) || (currentGamepad2.dpad_right && !previousGamepad2.dpad_right)) {
//            if (alliance == Alliance.red) {
                odo.set_heading(odo.get_heading(false)+2);
//            } else {
//                odo.set_heading(odo.get_heading(false)+1);
//            }
        } else if ((currentGamepad1.dpad_left && !previousGamepad1.dpad_left) || (currentGamepad2.dpad_left && !previousGamepad2.dpad_left)) {
//            if (alliance == Alliance.red) {
                odo.set_heading(odo.get_heading(false)-2);
//            } else {
//                Turret.blueShootPoint[1] -= 1;
//            }
        }

        if (flywheel.targetRPM != idleRpm && flywheel.targetRPM > 200 && flywheel.isReady() && turretReady) {
            gamepad1.rumble(500);
        }
        //if(!turret.autoAiming) turret.turret.setPower(-(gamepad1.dpad_left ? 0.6: 0) + (gamepad1.dpad_right ? 0.6: 0));
        //telemetry.addData("power", drivePower);
        //telemetry.addData("transferStage", armTransfer.transferStage);

//        telemetry.addData("Alliance", alliance);
//        telemetry.addData("Correction Drive?", useDriveCorrecton);
//        telemetry.addData("heading", odo.get_heading(false));
//        telemetry.update();

        double future_x = odo.get_x_predicted(false, false);
        double future_y = odo.get_y_predicted(false, false);

        double dist = Math.sqrt((future_x-target_shoot.x)*(future_x-target_shoot.x) + (future_y-target_shoot.y)*(future_y-target_shoot.y));
        dist = Math.min(dist, 108);

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
            packet.put("Turret Heading Reggin", turret.getAngle());
            packet.put("Odo heading", odo.get_heading(false));
            packet.put("Has ball", hasBackBall || hasMidBall);
            packet.put("in close zone", odo.inCloseZone(24));
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
