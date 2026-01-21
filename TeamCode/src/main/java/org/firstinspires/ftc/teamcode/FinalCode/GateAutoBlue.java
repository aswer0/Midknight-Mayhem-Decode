package org.firstinspires.ftc.teamcode.FinalCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
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
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.List;

@Autonomous
@Config
public class GateAutoBlue extends OpMode {
    public static Point start_point = new Point(25, 126);
    public static Point shoot_point = new Point(60, 81);
    public static Point park_point = new Point(20, 81);
    public static Point open_gate_point = new Point (14, 60);
    public static Point intake_gate_point = new Point(10, 56);

    public static double shootAngle = 180;
    public static double openGateAngle = 145;
    public static double intakeGateAngle = 115;

    public static double turretAngle = 44;

    BCPath gatePath = new BCPath(new Point[][] {
            {
                    new Point(60, 81),
                    new Point(48,69),
                    new Point(43,38.3),
//                    new Point(20.3, 69.5),
//                    new Point(8.3,56.1),
                    new Point(20,75),
                    new Point(10,59),

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
    public static ArrayList<Point> farBatch = new ArrayList<>(List.of(
            new Point(43.8,36.0),
            new Point(17.6,35.3)
    ));

    enum State{
        intakeBatch,
        intakeGate,
        driveToShootPos,
        shootBall,
        wait,
        park,
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

    public static boolean uk = false;
    public static double gvf_threshold = 0.67;
    public static double pid_threshold = 1.2;
    public static double power = 0.8;
    public static double first_shoot_wait_time = 2367;
    public static double shoot_wait_time = 1167;
    public static double gate_wait_time = 167;
    public static double intake_time = 1076.7;
    int loops = 0;
    public static int wait_time = 0;
    boolean do_path3 = true;

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
            state = State.park;
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
                if (loops != 4){
                    vf.move();
                }
                else{
                    at_point = pid_drive.pointDriver(180, 0.7, 1, pid_threshold, -1, uk, false);
                }

                if (vf.at_end(gvf_threshold) || (at_point && loops == 4)){
                    timer.reset();
                    state = State.driveToShootPos;
                }

                break;

            case intakeGate:
                vf.move();
                switch (gateState) {
                    case driveToGate:
                        intake.motorOff();
                        intake.doorClose();
                        //vf.move();
                        if (vf.at_end(1)) {
                            //wheelControl.stop();
                            timer.reset();
                            gateState = GateState.wait;
                        }
                        break;

                    case wait:
                        if (timer.milliseconds() > gate_wait_time || true) {
                            timer.reset();
                            gateState = GateState.intake;
                        }
                        break;

                    case intake:
                        intake.motorOn();
                        //wheelControl.drive_to_point(intake_gate_point, intakeGateAngle, 1, 0.5, false);
                        if (timer.milliseconds() > intake_time) {
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
                if (flywheel.isReady()) {
                    intake.doorOpen();
                    intake.motorOn();
                }

                wheelControl.drive_to_point(shoot_point, shootAngle, power, pid_threshold, uk);

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
                            vf.setPath(gatePath, intakeGateAngle, false);
                            pathPoints = gatePath.get_path_points();
                            gateState = GateState.driveToGate;
                            state = State.intakeGate;
                            break;
                        case 3:
                            vf.setPath(closeBatch, 180, false);
                            pathPoints = closeBatch.get_path_points();
                            state = State.intakeBatch;
                            break;
                        case 4:
                            state = State.intakeBatch;
                            break;
                    }
                }
                break;

            case park:
                flywheel.stop();
                intake.motorOff();
                wheelControl.drive_to_point(park_point, 180, 1, 0.5, false);
                break;
        }
    }
}
