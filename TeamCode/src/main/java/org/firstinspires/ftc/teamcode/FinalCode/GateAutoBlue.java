package org.firstinspires.ftc.teamcode.FinalCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Drivetrain.GVF.BCPath;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Drivetrain.GVF.VectorField;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Drivetrain.PIDdrive.Pathing;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Outtake.Flywheel;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Outtake.Turret;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Transfer.ArmTransfer;
import org.opencv.core.Mat;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.List;

@Autonomous(preselectTeleOp = "FinalTeleop")
@Config
public class GateAutoBlue extends OpMode {
    public static Point start_point = new Point(25, 126);
    public static Point shoot_point = new Point(60, 81);
    public static Point park_point = new Point(18, 81);
    public static Point park_shot_point = new Point(56.7,102);
    public static Point open_gate_point = new Point (14, 60);
    public static Point intake_gate_point = new Point(10, 55);

    public static double shootAngle = 180;
    public static double openGateAngle = 155;
    public static double intakeGateAngle = 115;

    public static double FIRST_TURRET_ANGLE = 43.8;
    public static double LAST_TURRET_ANGLE = 78.5;
    public  double turretAngle = FIRST_TURRET_ANGLE;
    public static double rpm = 2220;

    public static boolean uk = false;
    public static double gvf_threshold = 1;
    public static double pid_threshold = 1.2;
    public static double power = 1;
    public static double first_shoot_wait_time = 1867;
    public static double shoot_wait_time = 800;
    public static double gate_wait_time = 3667;
    public static double intake_time = 2000;
    int loops = 0;
    public static int wait_time = 0;
    public static boolean do_path3 = false;

    /*
    P_0 = (60, 81)
    P_1 = (48,69)
    P_2 = (43,38.3)
    P_3 = (20,75)
    P_4 = (10,59)

    * */

    BCPath gatePath = new BCPath(new Point[][] {
            {
                    new Point(60, 81),
                    new Point(52.6,72.7),
                    new Point(46.2,56.4),
                    new Point(10.2,59.2),

//                    new Point(48,69),
//                    new Point(43,38.3),
////                    new Point(20.3, 69.5),
////                    new Point(8.3,56.1),
//                    new Point(20,68.7),
//                    new Point(10,59),

//                    new Point (50.5,59.6),
//                    new Point(26.5,38.8),
//                    new Point(-32.9,106.1),
//                    new Point(38.5,46.3),
//                    new Point(3, 43),
//                    new Point(8.8, 64.6),
//                    new Point(8.3, 54.1),

//                    new Point(55,61.6),
//                    new Point(62,61.7),
//                    new Point(20,57.5),
//                    open_gate_point
            }
    });
    BCPath closeBatch = new BCPath(new Point[][] { //straight line
            {
                    new Point(60, 81),
                    new Point (20, 82.5),
                    //new Point(53.8,84.5),
                    //new Point(35,74.6),
                    //new Point(10.67,98.5),
                    //new Point(29.2,74.5),
                    //new Point(14.6,71.3),
            }
    });
    BCPath middleBatch = new BCPath(new Point[][] {
            {
                    new Point(60, 81),
                    new Point(55,61.6),
                    new Point(62,61.7),
                    new Point(20,57.5),
            }
    });
    BCPath parkShotPath = new BCPath(new Point[][] {
            {
                    new Point(17, 35.3),
                    park_shot_point,
            }
    });
    public static ArrayList<Point> farBatch = new ArrayList<>(List.of(
            new Point(43.8,36.0),
            new Point(14,35.3)
    ));

    enum State{
        intakeBatch,
        intakeGate,
        driveToShootPos,
        shootBall,
        wait,
        park,
        park_shot
    }

    enum GateState{
        driveToGate,
        wait,
        intake
    }

    State state = State.wait;
    GateState gateState = GateState.driveToGate;

    WheelControl wheelControl;
    Odometry odometry;
    VectorField vf;
    ElapsedTime timer;
    ElapsedTime autoTimer;

    Intake intake;
    Flywheel flywheel;
    Sensors sensors;
    ArmTransfer armTransfer;
    Turret turret;
    Pathing pid_drive;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();

    ArrayList<Point> pathPoints;

    @Override
    public void init() {
        odometry = new Odometry(hardwareMap, telemetry, start_point.x, start_point.y, 135);
        wheelControl = new WheelControl(hardwareMap, odometry);

        vf = new VectorField(wheelControl, odometry, uk);
        pid_drive = new Pathing(odometry, wheelControl);
        pid_drive.set_path(farBatch);
        timer = new ElapsedTime();
        autoTimer = new ElapsedTime();

        //pathPoints = presetPaths[loops+1].get_path_points();

        sensors = new Sensors(hardwareMap);
        intake = new Intake(hardwareMap, sensors);
        flywheel = new Flywheel(hardwareMap);
        armTransfer = new ArmTransfer(hardwareMap, intake);
        turret = new Turret(hardwareMap, null, odometry, FinalTeleop.Alliance.blue, true);
        FinalTeleop.alliance = FinalTeleop.Alliance.blue;

        flywheel.set_auto_coeffs();
    }

    @Override
    public void init_loop(){
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        if (!previousGamepad1.circle && currentGamepad1.circle){
            do_path3 = !do_path3;
        }
        if (!previousGamepad1.dpad_left && currentGamepad1.dpad_left){
            wait_time--;
            wait_time = Math.max(wait_time, 0);
        }
        if (!previousGamepad1.dpad_right && currentGamepad1.dpad_right){
            wait_time++;
        }

        telemetry.addData("do path 3? (circle)", do_path3);
        telemetry.addData("wait time (dpad)", wait_time);
        telemetry.update();
        turret.CURRENT_VOLTAGE = hardwareMap.voltageSensor.iterator().next().getVoltage();
    }

    @Override
    public void start() {
        timer.reset();
        autoTimer.reset();
        turret.setAngle(turretAngle);
        flywheel.shootClose();
    }

    @Override
    public void loop() {
        odometry.update();
        turret.update();
        flywheel.update();

        if (autoTimer.milliseconds() >= 29500) {
            //state = State.park;
            intake.doorClose();
        }

        switch (state) {
            case wait:
                if (timer.seconds() >= wait_time) {
                    state = State.driveToShootPos;
                }
                break;

            case intakeBatch:
                intake.doorClose();
                intake.motorOn();
                boolean at_point = false;
                if (loops != 4 || !do_path3){
                    vf.move();
                }
                else{
                    at_point = pid_drive.pointDriver(180, 1, 3, pid_threshold, -1, uk, false);
                }

                if (vf.at_end(gvf_threshold) || at_point){
                    if (loops == 5) {
                        timer.reset();
                        vf.setPath(parkShotPath, 225, false);
                        pathPoints = parkShotPath.get_path_points();
                        turretAngle = LAST_TURRET_ANGLE;
                        turret.setAngle(turretAngle);
                        state = State.park_shot;
                    } else {
                        timer.reset();
                        state = State.driveToShootPos;
                    }
                }
                break;

            case intakeGate:
                vf.move();
                switch (gateState) {
                    case driveToGate:
                        //intake.motorOff();
                        intake.motorOn();
                        intake.doorClose();
                        //vf.move();
                        if (vf.at_end(1) || timer.milliseconds() > gate_wait_time) {
                            //wheelControl.stop();
                            timer.reset();
                            gateState = GateState.intake;
                        }
                        break;

                    case wait:
                        break;

                    case intake:
                        intake.motorOn();
                        wheelControl.stop();
                        //wheelControl.drive_to_point(intake_gate_point, intakeGateAngle, 1, 0.5, false);
                        if (timer.milliseconds() > intake_time || intake.intakeCurrentThreshold(6.7) == 1) {
                            timer.reset();
                            state =State.driveToShootPos;
                        }
                        break;
                }
                break;

            case driveToShootPos:
                intake.motorOff();
                intake.doorOpen();
                if (wheelControl.drive_to_point(shoot_point, shootAngle, power, pid_threshold, uk) || timer.milliseconds() >= 3000){
                    timer.reset();
                    state = State.shootBall;
                }
                break;

            case shootBall:
                turret.setAngle(turretAngle);
                if (flywheel.isReady()) {
                    intake.doorOpen();
                    intake.motorOn();
                }

                if (loops >= 5) { //4
                    wheelControl.drive_to_point(park_shot_point, 225, 0.7, pid_threshold, uk);
                    FinalTeleop.startX = odometry.get_x(false);
                    FinalTeleop.startY = odometry.get_y(false);
                    FinalTeleop.startHeading = odometry.get_heading(false);
                } else {
                    wheelControl.drive_to_point(shoot_point, shootAngle, power, pid_threshold, uk);
                }

                if (timer.milliseconds() >= shoot_wait_time && loops != 0 || timer.milliseconds() >= first_shoot_wait_time) {
                    loops++;
                    timer.reset();
                    switch (loops) {
                        case 1:
                            vf.setPath(middleBatch, 180, false);
                            pathPoints = middleBatch.get_path_points();
                            state = State.intakeBatch;
                            break;
                        case 2:
                            vf.setPath(gatePath, openGateAngle, false);
                            pathPoints = gatePath.get_path_points();
                            gateState = GateState.driveToGate;
                            state = State.intakeGate;
                            break;
                        case 3:
                            vf.setPath(gatePath, openGateAngle, false);
                            pathPoints = gatePath.get_path_points();
                            gateState = GateState.driveToGate;
                            state = State.intakeGate;
                            break;
                        case 4:
                            if (do_path3) {
                                state = State.intakeBatch;
                            } else {
                                vf.setPath(gatePath, openGateAngle, false);
                                pathPoints = gatePath.get_path_points();
                                gateState = GateState.driveToGate;
                                state = State.intakeGate;
                            }
                            break;
                        case 5:
                            flywheel.setTargetRPM(rpm);
                            vf.setPath(closeBatch, 180, false);
                            pathPoints = closeBatch.get_path_points();
                            state = State.intakeBatch;
                            break;
                    }
                }
                break;

            case park:
                flywheel.stop();
                intake.motorOff();
                intake.doorClose();
                wheelControl.drive_to_point(park_point, 180, 1, 0.5, false);
                FinalTeleop.startX = odometry.get_x(false);
                FinalTeleop.startY = odometry.get_y(false);
                FinalTeleop.startHeading = odometry.get_heading(false);
                break;

            case park_shot:
                intake.motorOff();
                intake.doorOpen();
//                turretAngle = LAST_TURRET_ANGLE;

//                vf.move();
//                if (vf.at_end(gvf_threshold)){
                if (Math.abs(Math.abs(odometry.get_heading(false))-180) < 26.7) {
                    wheelControl.drive_to_point(new Point(36.7,90), 225, power, pid_threshold, false);
                } else {
                    if (wheelControl.drive_to_point(park_shot_point, 225, power, 2, false)) {
                        timer.reset();
                        flywheel.setTargetRPM(rpm);
                        state = State.shootBall;
                    }
                }
                break;
        }

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("x", odometry.get_x(uk));
        packet.put("y", odometry.get_y(uk));
        packet.put("heading", odometry.get_heading(uk));
        dashboard.sendTelemetryPacket(packet);
    }
}
