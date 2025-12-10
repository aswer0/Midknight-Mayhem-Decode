package org.firstinspires.ftc.teamcode.FinalCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experiments.DrivetrainExperiments.Camera;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Outtake.Turret;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Drivetrain.GVF.BCPath;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Drivetrain.GVF.VectorField;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Outtake.Flywheel;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Transfer.ArmTransfer;
import org.opencv.core.Point;

import java.util.ArrayList;

@Autonomous
@Config
public class CloseAutoBlueSide extends OpMode {
    public static Point start_point = new Point(25, 126);
    public static Point shoot_point = new Point(60, 81);

    /*
    P_0 = (60, 81)
    P_1 = (54,81.4)
    P_2 = (34.8,82.6)
    P_3 = (20,82.5)

    P_0 = (60, 81)
    P_1 = (54,81.4)
    P_2 = (34.8,82.6)
    P_3 = (25,82.5)

    P_0 = (52.6,102.4)
    P_1 = (50.2,72)
    P_2 = (62.6,27.7)
    P_3 = (53.7,39)
    P_4 = (25,38.6)

       ++++++++++++++++++

    P_0 = (60, 81)
    P_1 = (53.8,84.5)
    P_2 = (35,74.6)
    P_3 = (6,98.5)
    P_4 = (29.2,74.5)
    P_5 = (16,71.3)

    P_0 = (60, 81)
    P_1 = (55,64.8)
    P_2 = (62,61.7)
    P_3 = (23.7,59.6)

    P_0 = (52.6,102.4)
    P_1 = (50.2,72)
    P_2 = (62.6,27.7)
    P_3 = (53.7,39)
    P_4 = (25,34)

    P_0 = (60, 81)
    P_1 = (55,64.8)
    P_2 = (62,61.7)
    P_3 = (23.7,59.6)

     */

    public static BCPath[] follow_paths = {
            new BCPath(new Point[][] {
                    {
                            new Point(60, 81),
                            new Point(53.8,84.5),
                            new Point(35,74.6),
                            new Point(6,98.5),
                            new Point(29.2,74.5),
                            new Point(16,71.3),
                    }
            }),
            new BCPath(new Point[][] {
                    {
                            new Point(60, 81),
                            new Point(55,61.6),
                            new Point(62,61.7),
                            new Point(24,57.5),
                    }
            }),
            new BCPath(new Point[][] {
                    {
                            new Point(52.6,102.4),
                            new Point(50.2,72),
                            new Point(62.6,27.7),
                            new Point(53.7,39),
                            new Point(25,34),
                    }
            })
    };

    enum State{
        intakeBatch,
        driveToShootPos,
        shootBall,
        wait,
        park,
    }

    State state = State.wait;

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

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();

    public static boolean uk = false;
    public static double gvf_threshold = 0.5;
    public static double pid_threshold = 1.2;
    public static double power = 0.8;
    public static double turret_angle = 44;
    public double shoot_angle = 135;

    public static double shoot_wait_time = 3750;
    public static double gate_wait_time = 750;

    int loops = -1;
    public static int wait_time = 0;
    boolean do_path3 = true;
    int shotCounter = 0;
    boolean prevFlywheelReady = false;

    ArrayList<Point> pathPoints;

    @Override
    public void init() {
        odometry = new Odometry(hardwareMap, telemetry, start_point.x, start_point.y, 135);
        wheelControl = new WheelControl(hardwareMap, odometry);

        vf = new VectorField(wheelControl, odometry, uk);
        timer = new ElapsedTime();
        autoTimer = new ElapsedTime();

        pathPoints = follow_paths[loops+1].get_path_points();

        sensors = new Sensors(hardwareMap);
        intake = new Intake(hardwareMap, sensors);
        flywheel = new Flywheel(hardwareMap);
        armTransfer = new ArmTransfer(hardwareMap, intake);
        turret = new Turret(hardwareMap, new Camera(hardwareMap), odometry, FinalTeleop.Alliance.blue, true);
        FinalTeleop.alliance = FinalTeleop.Alliance.blue;
    }

    @Override
    public void init_loop(){
        timer.reset();
        autoTimer.reset();

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
    }

    @Override
    public void loop() {
        odometry.update();
        turret.update();
        boolean isTransferReady = armTransfer.update();

        if (autoTimer.milliseconds() >= 29500){
            state = State.park;
        }

        switch (state){
            case wait:
                if (timer.milliseconds() >= wait_time){
                    state = State.driveToShootPos;
                }
                break;

            case intakeBatch:
                turret.setAngle(turret_angle);
                armTransfer.toIdle();
                flywheel.setTargetRPM(0);
                flywheel.update();
                intake.motorOn();

                vf.move();

                if (loops == 0){
                    if (timer.milliseconds() >= 2867){
                        timer.reset();
                        state = State.driveToShootPos;
                    }
                }
                if (vf.at_end(gvf_threshold)){
                    timer.reset();
                    state = State.driveToShootPos;
                }
                break;

            case driveToShootPos:
                armTransfer.toIdle();
                intake.motorOff();
                flywheel.shootClose();
                flywheel.update();

                if (loops == 0){
                    if (timer.milliseconds() <= gate_wait_time){
                        wheelControl.stop();
                        break;
                    }
                }

                if (wheelControl.drive_to_point(shoot_point, shoot_angle, power, pid_threshold, uk) || timer.milliseconds() >= 3000){
                    timer.reset();
                    shotCounter = 0;
                    state = State.shootBall;
                }
                break;

            case shootBall:
                flywheel.shootClose();
                flywheel.update();
                if (flywheel.isReady()){
                    if (isTransferReady) {
                        armTransfer.transfer();
                        //shotCounter++;
                    }
                }

                if (flywheel.isReady() && !prevFlywheelReady) {
                    shotCounter++;
                }
                prevFlywheelReady = flywheel.isReady();

                wheelControl.drive_to_point(shoot_point, shoot_angle, power, pid_threshold, uk);

                if (shotCounter > 3 || timer.milliseconds() >= shoot_wait_time){
                    loops++;

                    if (loops >= 2+(do_path3 ? 1 : 0)){
                        state = State.park;
                    }
                    else {
                        vf.setPath(follow_paths[loops], 180, false);
                        if (loops >= 2){
                            //vf.setPath(follow_paths[loops], 180, true);
                        }
                        pathPoints = follow_paths[loops].get_path_points();

                        armTransfer.toIdle();
                        armTransfer.current_shots = 0;
                        shoot_angle = 180;
                        timer.reset();
                        state = State.intakeBatch;
                    }
                }
                break;

            case park:
                armTransfer.toIdle();
                flywheel.update();
                flywheel.stop();
                intake.motorOff();

                wheelControl.drive_to_point(new Point(20, 81), 180, 1, 0.5, false);
                break;
        }

        TelemetryPacket packet = new TelemetryPacket();

        packet.fieldOverlay() //draw robot
                .setTranslation(-72, 72)
                .setRotation(Math.toRadians(-90))
                .setStroke("blue")
                .strokeCircle(odometry.get_x(uk), odometry.get_y(uk), 15/2)
                .strokeLine(odometry.get_x(uk), odometry.get_y(uk),odometry.get_x(uk) + 15/2*Math.cos(odometry.get_heading(uk)), odometry.get_y(uk) + 15/2*Math.sin(odometry.get_heading(uk)));

        packet.fieldOverlay() //draw target path
                .setFill("red")
                .setStroke("orange")
                .fillCircle(start_point.x, start_point.y, 2)
                .strokeLine(start_point.x, start_point.y, pathPoints.get(0).x, pathPoints.get(0).y);

        for (int i=0; i<pathPoints.size()-1; i++) {
            packet.fieldOverlay()
                    .strokeLine(pathPoints.get(i).x, pathPoints.get(i).y, pathPoints.get(i + 1).x, pathPoints.get(i + 1).y);
        }

        packet.put("state", state);
        packet.put("x", odometry.get_x(uk));
        packet.put("y", odometry.get_y(uk));
        packet.put("heading", odometry.get_heading(uk));
        packet.put("RPM", flywheel.getCurrentRPM());

        dashboard.sendTelemetryPacket(packet);
        telemetry.addData("shot counter", shotCounter);
    }
}
