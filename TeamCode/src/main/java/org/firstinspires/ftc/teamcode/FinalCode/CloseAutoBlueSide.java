package org.firstinspires.ftc.teamcode.FinalCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experiments.Sensors;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Drivetrain.GVF.BCPath;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Drivetrain.GVF.VectorField;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake.Flywheel;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Transfer.ArmTransfer;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Transfer.BeltTransfer;
import org.opencv.core.Point;

import java.util.ArrayList;

@Autonomous
@Config
public class CloseAutoBlueSide extends OpMode {
    public static Point start_point = new Point(25, 126);
    public static Point shoot_point = new Point(60, 81);

    /*
    P_0 = (52.6,102.4)
    P_1 = (50.2,72)
    P_2 = (62.6,27.7)
    P_3 = (53.7,35.7)
    P_4 = (20,35.2)

     = (60, 81)
     = (45.3,73)
     = (62.4,55.7)
     = (20,59)
     */

    BCPath[] follow_paths = {
            new BCPath(new Point[][] {
                    {
                            new Point(60, 81),
                            new Point(54,81.4),
                            new Point(34.8,82.6),
                            new Point(20,82.5),
                    }
            }),
            new BCPath(new Point[][] {
                    {
                            new Point(60, 81),
                            new Point(54.7,71.6),
                            new Point(62.4,55.7),
                            new Point(19,62.1),
                    }
            }),
            new BCPath(new Point[][] {
                    {
                            new Point(52.6,102.4),
                            new Point(50.2,72),
                            new Point(62.6,27.7),
                            new Point(53.7,39),
                            new Point(19.4,38.6),
                    }
            })
    };

    enum State{
        intakeBatch,
        driveToShootPos,
        shootBall,
        wait,
    }

    State state = State.wait;

    WheelControl wheelControl;
    Odometry odometry;
    VectorField vf;
    BCPath path;
    ElapsedTime timer;

    Intake intake;
    Flywheel flywheel;
    Sensors sensors;
    BeltTransfer beltTransfer;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();

    public static boolean uk = false;
    public static double gvf_threshold = 0.5;
    public static double pid_threshold = 0.8;
    public static double shoot_angle = 135;
    public static double power = 0.8;
    int loops = -1;
    int shots = 0;

    int wait_time = 0;

    boolean do_path1 = true;
    boolean do_path2 = true;
    boolean do_path3 = true;

    ArrayList<Point> pathPoints;

    @Override
    public void init() {
        odometry = new Odometry(hardwareMap, telemetry, start_point.x, start_point.y, 135);
        wheelControl = new WheelControl(hardwareMap, odometry);
        beltTransfer = new BeltTransfer(hardwareMap);

        vf = new VectorField(wheelControl, odometry, uk);
        timer = new ElapsedTime();

        pathPoints = follow_paths[loops+1].get_path_points();

        sensors = new Sensors(hardwareMap);
        intake = new Intake(hardwareMap, sensors);
        flywheel = new Flywheel(hardwareMap);
    }

    @Override
    public void init_loop(){
        timer.reset();

        if (!previousGamepad1.square && currentGamepad1.square){
            do_path1 = !do_path1;
        }
        if (!previousGamepad1.triangle && currentGamepad1.triangle){
            do_path2 = !do_path2;
        }
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

        telemetry.addData("do path 1? (square)", do_path1);
        telemetry.addData("do path 2? (triangle)", do_path2);
        telemetry.addData("do path 3? (circle)", do_path3);
        telemetry.addData("wait time (dpad)", wait_time);
        telemetry.update();
    }

    @Override
    public void loop() {
        odometry.update();

        switch (state){
            case wait:
                if (timer.milliseconds() >= wait_time){
                    state = State.driveToShootPos;
                }
                break;

            case intakeBatch:
                flywheel.stop();
                beltTransfer.stop();
                intake.motorOff();

                if (loops >= 3){
                    state = State.intakeBatch;
                }

                vf.move();
                intake.motorOn();

                if (vf.at_end(gvf_threshold)){
                    state = State.driveToShootPos;
                }
                break;

            case driveToShootPos:
                intake.motorOff();

                if (wheelControl.drive_to_point(shoot_point, shoot_angle, power, pid_threshold, uk)){
                    timer.reset();
                    state = State.shootBall;
                }
                break;

            case shootBall:
                flywheel.shootClose();
                flywheel.update();
                intake.motorOn();
                beltTransfer.up();

                wheelControl.setI();
                wheelControl.drive_to_point(shoot_point, shoot_angle, power, pid_threshold, uk);

                if (timer.milliseconds() >= 5000){
                    loops++;
                    if (loops == 0 && !do_path1){
                        loops++;
                    }
                    if (loops == 1 && !do_path2){
                        loops++;
                    }
                    if (loops == 2 && !do_path3){
                        loops++;
                    }

                    wheelControl.zeroI();
                    vf.setPath(follow_paths[loops], 180, true);
                    pathPoints = follow_paths[loops].get_path_points();

                    shots = 0;
                    state = State.intakeBatch;
                }

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

        dashboard.sendTelemetryPacket(packet);
    }
}
