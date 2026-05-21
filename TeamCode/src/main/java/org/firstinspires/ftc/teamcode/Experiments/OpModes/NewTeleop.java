//package org.firstinspires.ftc.teamcode.FinalCode;
//
//import static org.firstinspires.ftc.teamcode.FinalCode.Constants.blueGatePoint;
//import static org.firstinspires.ftc.teamcode.FinalCode.Constants.blueResetPoint;
//import static org.firstinspires.ftc.teamcode.FinalCode.Constants.gateAngle;
//import static org.firstinspires.ftc.teamcode.FinalCode.Constants.redGatePoint;
//import static org.firstinspires.ftc.teamcode.FinalCode.Constants.redResetPoint;
//import static org.firstinspires.ftc.teamcode.FinalCode.Constants.shootWaitTime;
//import static org.firstinspires.ftc.teamcode.FinalCode.Constants.slowTransferOffTime;
//import static org.firstinspires.ftc.teamcode.FinalCode.Constants.slowTransferOnTime;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDFCoefficients;
//import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Drivetrain.Odometry;
//import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Drivetrain.WheelControl;
//import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Intake.Intake;
//import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Outtake.Flywheel;
//import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Outtake.Turret;
//import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Sensors;
//import org.opencv.core.Point;
//
//@TeleOp
//@Config
//public class NewTeleop extends OpMode {
//    Odometry odo;
//    WheelControl drive;
//    Sensors sensors;
//    Intake intake;
//    Flywheel flywheel;
//    Turret turret;
//
//    Gamepad currentGamepad1 = new Gamepad();
//    Gamepad previousGamepad1 = new Gamepad();
//    Gamepad currentGamepad2 = new Gamepad();
//    Gamepad previousGamepad2 = new Gamepad();
//
//    public static PIDFCoefficients turretCoefficients = new PIDFCoefficients(0.02, 0.003, 0.00025,0.2);
//    public static int idleRpm = 2467;
//
//    int x_sign;
//    int y_sign;
//    Point target_shoot = new Point(0, 0);
//
//    public static Constants.Alliance alliance = Constants.Alliance.red;
//    public static double startX = 8;
//    public static double startY = 8;
//    public static double startHeading = 0.0;
//    boolean useDriveCorrection = true;
//    boolean gateCycleMode = false;
//    boolean pidToGate = false;
//    boolean shootFar = false;
//    public static boolean outputDebugInfo = false;
//    FtcDashboard dashboard = FtcDashboard.getInstance();
//
//    ElapsedTime doorTimer = new ElapsedTime();
//    ElapsedTime transferTimer = new ElapsedTime();
//
//    @Override
//    public void init() {
//        odo = new Odometry(hardwareMap, telemetry, startX, startY, startHeading);
//        drive = new WheelControl(hardwareMap, odo);
//        sensors = new Sensors(hardwareMap);
//        intake = new Intake(hardwareMap, sensors);
//        flywheel = new Flywheel(hardwareMap);
//        turret = new Turret(hardwareMap, null, odo, alliance, false, turretCoefficients);
//
//        flywheel.CURRENT_VOLTAGE = hardwareMap.voltageSensor.iterator().next().getVoltage();
//        flywheel.use_gained_schedule = true;
//        flywheel.set_tele_coeffs();
//
//        turret.CURRENT_VOLTAGE = hardwareMap.voltageSensor.iterator().next().getVoltage();
//        turret.autoAiming = true;
//    }
//
//    @Override
//    public void init_loop() {
//        telemetry.addData("Alliance", alliance);
//        telemetry.update();
//        previousGamepad1.copy(currentGamepad1);
//        currentGamepad1.copy(gamepad1);
//
//        if (currentGamepad1.triangle && !previousGamepad1.triangle){
//            if (alliance == Constants.Alliance.blue)  alliance = Constants.Alliance.red;
//            else                            alliance = Constants.Alliance.blue;
//        }
//
//        if (alliance == Constants.Alliance.red){
//            x_sign = 1;
//            y_sign = 1;
//        } else {
//            x_sign = -1;
//            y_sign = -1;
//        }
//        odo.set_x(startX);
//        odo.set_y(startY);
//
//        turret.alliance = alliance;
//    }
//
//    @Override
//    public void start() {
//        doorTimer.reset();
//    }
//
//    @Override
//    public void loop() {
//        previousGamepad1.copy(currentGamepad1);
//        currentGamepad1.copy(gamepad1);
//        previousGamepad2.copy(currentGamepad2);
//        currentGamepad2.copy(gamepad2);
//        odo.update();
//        flywheel.update();
//
//        if (currentGamepad1.options && !previousGamepad1.options) {
//            if (alliance == Constants.Alliance.red){
//                odo.set_heading(0);
//                odo.set_x(redResetPoint.x);
//                odo.set_y(redResetPoint.y);
//            }
//            else if (alliance == Constants.Alliance.blue){
//                odo.set_heading(180);
//                odo.set_x(blueResetPoint.x);
//                odo.set_y(blueResetPoint.y);
//            }
//        }
//
//        if (currentGamepad1.share && !previousGamepad1.share) {
//            useDriveCorrection = !useDriveCorrection;
//        }
//        pidToGate = currentGamepad1.left_stick_button;
//
//        //intake
//        if (currentGamepad1.right_bumper) { //in
//            if (sensors.hasAllBalls()) {
//                intake.motorOff();
//                gamepad1.rumble(500);
//            } else {
//                intake.motorOn();
//                intake.doorClose();
//                doorTimer.reset();
//            }
//        } else if (currentGamepad1.right_trigger > 0.3) { //reverse
//            intake.motorReverse();
//        } else {
//            if (doorTimer.milliseconds() > 400) intake.doorOpen();
//            if (currentGamepad1.left_bumper) { //transfer
//                if (transferTimer.milliseconds() > shootWaitTime) {
//                    doorTimer.reset();
//                    intake.doorClose();
//                    intake.motorOn();
//                    if (gateCycleMode) pidToGate = true;
//                } else {
//                    intake.doorOpen();
//                    intake.motorOn();
//                }
//            } else if (currentGamepad1.left_trigger > 0.3) { //slow transfer
//                intake.doorOpen();
//                intake.intervalTransfer(transferTimer.milliseconds(), slowTransferOnTime, slowTransferOffTime);
//            } else {
//                intake.motorOff();
//                transferTimer.reset();
//            }
//        }
//
//        //shooter
//        double future_x = odo.get_x_predicted(false, false);
//        double future_y = odo.get_y_predicted(false, false);
//
//        double dist = Math.hypot(future_x-target_shoot.x, future_y-target_shoot.y);
//
//        if (odo.inCloseZone(24) || (sensors.hasBackBall() || sensors.hasMidBall())) {
//            if (dist > 115 && shootFar) {
//                flywheel.set_auto_far_rpm(dist);
//                flywheel.shootAutoDist();
//            } else {
//                dist = Math.min(dist, 115);
//                flywheel.set_auto_hood_rpm(dist);
//                flywheel.shootAutoDist();
//            }
//        } else {
//            flywheel.setTargetRPM(idleRpm);
//        }
//
//        if (flywheel.targetRPM != idleRpm && flywheel.targetRPM > 200 && flywheel.isReady()) {
//            gamepad1.rumble(500);
//        }
//
//        //drive
//        if (useDriveCorrection && !pidToGate){
//            drive.correction_drive(x_sign*gamepad1.left_stick_y, y_sign*1.2 * gamepad1.left_stick_x, -gamepad1.right_stick_x, -Math.toRadians(-odo.get_heading(false)), 1, false);
//        } else if (!pidToGate){
//            drive.drive(x_sign*gamepad1.left_stick_y, y_sign*1.2 * gamepad1.left_stick_x, -gamepad1.right_stick_x, -Math.toRadians(-odo.get_heading(false)), 1);
//        } else{
//            if (alliance == Constants.Alliance.blue) {
//                drive.drive_to_point(blueGatePoint, 180 - gateAngle, 0.67, 0.5, false);
//            } else {
//                drive.drive_to_point(redGatePoint, gateAngle, 0.67, 0.5, false);
//            }
//        }
//
//        if (outputDebugInfo) {
//            TelemetryPacket packet = new TelemetryPacket();
//            packet.put("distance", dist);
//            packet.put("x", odo.get_x(false));
//            packet.put("y", odo.get_y(false));
//            packet.put("Odo heading", odo.get_heading(false));
//            packet.put("Auto RPM", Flywheel.AUTO_RPM);
//            packet.put("RPM", flywheel.getCurrentRPM());
//            packet.put("Turret Heading", turret.getAngle());
//            packet.put("Has ball", sensors.hasBackBall() || sensors.hasMidBall());
//            packet.put("in close zone", odo.inCloseZone(24));
//
//            dashboard.sendTelemetryPacket(packet);
//        }
//    }
//}
