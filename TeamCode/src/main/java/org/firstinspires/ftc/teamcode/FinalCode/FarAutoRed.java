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

@Autonomous
@Config
public class FarAutoRed extends OpMode {
    public static Point start_point = new Point(88, 8);
    public static Point park_point = new Point(110, 10);

    public static BCPath[] follow_paths = {
            new BCPath(new Point[][]{
                    {
                            new Point(144-56, 8),
                            new Point(144-22, 9),
                            new Point(144-8, 1.3),
                            new Point(144-17.4,15.3),
                            new Point(144-18.5,13),
                            new Point(144-30, 18),
                            new Point(144-8,8),
                    }
            })
    };

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
    public static double turret_angle = -67.67;
    public static double bot_angle = 0;

    public static double shoot_wait_time = 6000;

    int loops = 0;
    int wait_time = 0;
    int shotCounter = 0;
    boolean prevFlywheelReady = false;

    ArrayList<Point> pathPoints;

    @Override
    public void init() {
        odometry = new Odometry(hardwareMap, telemetry, start_point.x, start_point.y, bot_angle);
        wheelControl = new WheelControl(hardwareMap, odometry);

        vf = new VectorField(wheelControl, odometry, uk);
        timer = new ElapsedTime();

        pathPoints = follow_paths[loops].get_path_points();

        sensors = new Sensors(hardwareMap);
        intake = new Intake(hardwareMap, sensors);
        flywheel = new Flywheel(hardwareMap);
        armTransfer = new ArmTransfer(hardwareMap, intake);
        turret = new Turret(hardwareMap, null, odometry, FinalTeleop.Alliance.red, true);
        FinalTeleop.alliance = FinalTeleop.Alliance.red;
        turret.CURRENT_VOLTAGE = hardwareMap.voltageSensor.iterator().next().getVoltage();
    }

    @Override
    public void init_loop() {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        timer.reset();

        if (!previousGamepad1.dpad_left && currentGamepad1.dpad_left){
            wait_time--;
            wait_time = Math.max(wait_time, 0);
        }
        if (!previousGamepad1.dpad_right && currentGamepad1.dpad_right){
            wait_time++;
        }

        telemetry.addData("wait time (dpad)", wait_time);
        telemetry.update();
    }

    @Override
    public void start() {
        turret.setAngle(turret_angle);
    }

    @Override
    public void loop() {
        odometry.update();
        turret.update();
        boolean isTransferReady = true; armTransfer.update();

        switch (state) {
            case wait:
                flywheel.shootFar();
                flywheel.update();
                if (timer.seconds() >= wait_time) {
                    timer.reset();
                    state = State.shootBall;
                }
                break;

            case shootBall:
                flywheel.shootFar();
                flywheel.update();

                if (flywheel.isReady()) {
                    if (isTransferReady) {
                        intake.doorOpen();
                        intake.motorOn();
//                        armTransfer.transfer();
                    }
                }

                wheelControl.drive_to_point(start_point, bot_angle, power, pidf_threshold, uk);

                if (timer.milliseconds() >= shoot_wait_time) {
                    loops++;

                    if (loops > 1) {
                        state = State.park;
                    } else {
                        vf.setPath(follow_paths[loops-1], 0, false);
                        pathPoints = follow_paths[loops-1].get_path_points();
//                        armTransfer.toIdle();
//                        armTransfer.current_shots = 0;
                        timer.reset();
                        state = State.intakeBatch;
                    }
                }

                break;

            case intakeBatch:
                flywheel.shootFar();
                flywheel.update();
                intake.motorOn();

                vf.move();

                if (vf.at_end(gvf_threshold) || timer.milliseconds() > 2867) {
                    timer.reset();
                    state = State.driveToShootPos;
                }
                break;

            case driveToShootPos:
                intake.motorOff();
                flywheel.shootFar();
                flywheel.update();

                if (wheelControl.drive_to_point(start_point, bot_angle, power, pidf_threshold, uk)) {
                    timer.reset();
                    shotCounter = 0;
                    state = State.shootBall;
                }
                break;

            case park:
//                armTransfer.toIdle();
                flywheel.stop();
                flywheel.update();
                intake.motorOn();

                wheelControl.drive_to_point(park_point, bot_angle, 1, pidf_threshold, false);

                FinalTeleop.startX = park_point.x;
                FinalTeleop.startY = park_point.y;
                FinalTeleop.startHeading = bot_angle;
                break;
        }

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("turret angle (actual)", turret.getAngle());
    }
}
