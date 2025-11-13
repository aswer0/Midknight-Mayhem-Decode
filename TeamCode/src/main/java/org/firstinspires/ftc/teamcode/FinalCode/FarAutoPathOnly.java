package org.firstinspires.ftc.teamcode.FinalCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experiments.Sensors;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Drivetrain.GVF.BCPath;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Drivetrain.GVF.VectorField;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake.Flywheel;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Transfer.ArmTransfer;
import org.opencv.core.Point;

import java.util.ArrayList;

@Autonomous
@Config
public class FarAutoPathOnly extends OpMode {
    public static Point start_point = new Point(55, 8);
    public static Point shoot_point = new Point(58, 21);

    /*
    P_0 = (52.6, 7.6)
    P_1 = (45.5, 23.4)
    P_2 = (60, 35)
    P_3 = (18.4, 34.6)

    P_0 = (52.6, 7.6)
    P_1 = (45.5, 23.4)
    P_2 = (63.6, 66.2)
    P_3 = (10.2, 58.2)
     */
    BCPath[] follow_paths = {
            new BCPath(new Point[][] {
                    {
                            new Point(58, 21),
                            new Point(46, 5),
                            new Point(27.6, 13),
                            new Point(13, 11)
                    }
            }),
            new BCPath(new Point[][] {
                    {
                            new Point(52.6, 7.6),
                            new Point(45.5, 23.4),
                            new Point(60, 35),
                            new Point(18.4, 34.6)
                    }
            }),
            new BCPath(new Point[][] {
                    {
                            new Point(52.6, 7.6),
                            new Point(45.5, 23.4),
                            new Point(63.6, 66.2),
                            new Point(19.6,58.6)
                    }
            }),
    };

    enum State{
        intakeBatch,
        driveToShootPos,
        shootBall,
    }

    State state = State.driveToShootPos;

    WheelControl wheelControl;
    Odometry odometry;
    VectorField vf;
    BCPath path;
    ElapsedTime timer;

    Intake intake;
    Flywheel flywheel;
    Sensors sensors;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    public static boolean uk = false;
    public static double gvf_threshold = 0.5;
    public static double pid_threshold = 1.2;
    public static double shoot_angle = 113;
    public static double power = 0.8;
    int loops = -1;

    ArrayList<Point> pathPoints;

    @Override
    public void init() {
        odometry = new Odometry(hardwareMap, telemetry, start_point.x, start_point.y, 90);
        wheelControl = new WheelControl(hardwareMap, odometry);

        vf = new VectorField(wheelControl, odometry, uk);
        timer = new ElapsedTime();

        sensors = new Sensors(hardwareMap);
        intake = new Intake(hardwareMap, sensors);
        flywheel = new Flywheel(hardwareMap);

        pathPoints = follow_paths[loops+1].get_path_points();
    }

    @Override
    public void init_loop(){
        timer.reset();
    }

    @Override
    public void loop() {
        odometry.update();

        switch (state){
            case intakeBatch:
                vf.move();

                if (vf.at_end(gvf_threshold)){
                    state = State.driveToShootPos;
                }
                break;

            case driveToShootPos:
                if (wheelControl.drive_to_point(shoot_point, shoot_angle, power, pid_threshold, uk)){
                    timer.reset();
                    state = State.shootBall;
                }
                break;

            case shootBall:
                wheelControl.drive_to_point(shoot_point, shoot_angle, power, pid_threshold, uk);
                if (timer.milliseconds() >= 5000){
                    loops++;
                    vf.setPath(follow_paths[loops], 180, false);
                    pathPoints = follow_paths[loops].get_path_points();

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
