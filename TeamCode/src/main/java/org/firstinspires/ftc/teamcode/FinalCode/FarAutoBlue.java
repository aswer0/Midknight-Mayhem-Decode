package org.firstinspires.ftc.teamcode.FinalCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experiments.DrivetrainExperiments.Camera;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Drivetrain.GVF.BCPath;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Drivetrain.GVF.VectorField;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Outtake.Flywheel;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Outtake.Turret;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Transfer.ArmTransfer;
import org.opencv.core.Point;

import java.util.ArrayList;

@Autonomous(preselectTeleOp = "FinalTeleop")
@Config
public class FarAutoBlue extends OpMode {
    public static Point start_point = new Point(56, 8);
    public static Point shoot_point = new Point(56, 16);
    public static Point park_point = new Point(30, 10);

    BCPath cornerPath = new BCPath(new Point[][]{
            {
                    new Point(56, 16),
                    new Point(22, 9),
                    new Point(8, 1.3),
                    new Point(17.4,15.3),
                    new Point(18.5,13),
                    new Point(30, 18),
                    new Point(6.5,8),
            }
    });

    BCPath preloadPath = new BCPath(new Point[][]{
            {
                    new Point(56, 16),
                    new Point(55.5, 20.6),
                    new Point(59, 37.8),
                    new Point(16.4, 35),
            }
    });

    enum State {
        wait,
        shootBall,
        intakeBatch,
        driveToShootPos,
        park
    }

    State state = State.wait;

    WheelControl wheelControl;
    Odometry odometry;
    VectorField vf;
    ElapsedTime timer;
    ElapsedTime parkTimer;
    ElapsedTime transferTimer;

    Intake intake;
    Flywheel flywheel;
    Sensors sensors;
    ArmTransfer armTransfer;
    Turret turret;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();

    public static boolean uk = false;
    public static double gvf_threshold = 0.67;
    public static double pidf_threshold = 0.5;
    public static double power = 0.8;
    public static double turret_angle = 66.5;
    public static double bot_angle = 180;
    public static double first_shoot_wait_time = 4000;
    public static double shoot_wait_time = 2000;
    public static int numCornerCycles = 5;
    public static double transferOnTime = 200;
    public static double transferOffTime = 400;
    public static boolean do_path_3 = true;

    int loops = 0;
    int wait_time = 0;

    ArrayList<Point> pathPoints;

    @Override
    public void init() {
        odometry = new Odometry(hardwareMap, telemetry, start_point.x, start_point.y, bot_angle);
        wheelControl = new WheelControl(hardwareMap, odometry);

        vf = new VectorField(wheelControl, odometry, uk);
        timer = new ElapsedTime();
        parkTimer = new ElapsedTime();
        transferTimer = new ElapsedTime();

        pathPoints = cornerPath.get_path_points();

        sensors = new Sensors(hardwareMap);
        intake = new Intake(hardwareMap, sensors);
        flywheel = new Flywheel(hardwareMap);
        armTransfer = new ArmTransfer(hardwareMap, intake);
        turret = new Turret(hardwareMap, null, odometry, FinalTeleop.Alliance.blue, true);
        FinalTeleop.alliance = FinalTeleop.Alliance.blue;

        flywheel.set_tele_coeffs();
    }

    @Override
    public void init_loop() {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        if (!previousGamepad1.circle && currentGamepad1.circle){
            do_path_3 = !do_path_3;
        }
        if (!previousGamepad1.dpad_left && currentGamepad1.dpad_left){
            wait_time--;
            wait_time = Math.max(wait_time, 0);
        }
        if (!previousGamepad1.dpad_right && currentGamepad1.dpad_right){
            wait_time++;
        }

        telemetry.addData("do path 3? (circle)", do_path_3);
        telemetry.addData("wait time (dpad)", wait_time);
        telemetry.update();
        turret.CURRENT_VOLTAGE = hardwareMap.voltageSensor.iterator().next().getVoltage();
    }

    @Override
    public void start() {
        timer.reset();
        parkTimer.reset();
        transferTimer.reset();
        turret.setAngle(turret_angle);
    }

    @Override
    public void loop() {
        odometry.update();
        turret.update();
        flywheel.update();

        if (parkTimer.milliseconds() > 29500) {
            state = State.park;
        }

        switch (state) {
            case wait:
                flywheel.shootFar();
                if (timer.seconds() >= wait_time) {
                    timer.reset();
                    state = State.driveToShootPos;
                }
                break;

            case shootBall:
                flywheel.shootFar();

                if (flywheel.isReady()) {
                    intake.doorOpen();
                    intake.intervalTransfer(timer.milliseconds(), transferOnTime, transferOffTime);
                } else {
                    transferTimer.reset();
                }

                wheelControl.drive_to_point(shoot_point, bot_angle, power, pidf_threshold, uk);

                if (timer.milliseconds() >= shoot_wait_time && loops != 0 || timer.milliseconds() >= first_shoot_wait_time) {
                    intake.doorClose();
                    loops++;

                    if (loops == 1 && do_path_3) {
                        vf.setPath(preloadPath, 180, false);
                        pathPoints = preloadPath.get_path_points();
                        timer.reset();
                        state = State.intakeBatch;
                    } else if (loops > numCornerCycles || parkTimer.milliseconds() > 25000) {
                        state = State.park;
                    } else {
                        vf.setPath(cornerPath, 180, false);
                        pathPoints = cornerPath.get_path_points();
                        timer.reset();
                        state = State.intakeBatch;
                    }
                }

                break;

            case intakeBatch:
                flywheel.shootFar();
                intake.motorOn();
                intake.doorClose();

                vf.move();

                if (vf.at_end(gvf_threshold) || timer.milliseconds() > 2867) {
                    timer.reset();
                    state = State.driveToShootPos;
                }
                break;

            case driveToShootPos:
                intake.motorOff();
                flywheel.shootFar();

                if (wheelControl.drive_to_point(shoot_point, bot_angle, power, pidf_threshold, uk)) {
                    timer.reset();
                    transferTimer.reset();
                    state = State.shootBall;
                }
                break;

            case park:
                intake.motorOff();
                flywheel.stop();

                wheelControl.drive_to_point(park_point, bot_angle, 1, pidf_threshold, false);

                FinalTeleop.startX = park_point.x;
                FinalTeleop.startY = park_point.y;
                FinalTeleop.startHeading = bot_angle;
                break;
        }

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("turret angle (actual)", turret.getAngle());
        dashboard.sendTelemetryPacket(packet);
    }
}
