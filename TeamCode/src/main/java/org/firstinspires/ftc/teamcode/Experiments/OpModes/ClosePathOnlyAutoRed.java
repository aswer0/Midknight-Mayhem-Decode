package org.firstinspires.ftc.teamcode.Experiments.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Drivetrain.GVF.BCPath;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Drivetrain.GVF.VectorField;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Drivetrain.WheelControl;
import org.opencv.core.Point;

import java.util.ArrayList;

@Autonomous
@Config
public class ClosePathOnlyAutoRed extends OpMode {
    public static Point start_point = new Point(142-25, 126);
    public static Point shoot_point = new Point(142-60, 81);


    public static BCPath[] follow_paths = {
            new BCPath(new Point[][] {
                    {
                            new Point(142-60, 81),
                            new Point(142-53.8,84.5),
                            new Point(142-35,74.6),
                            new Point(142-6,98.5),
                            new Point(142-29.2,74.5),
                            new Point(142-16,71.3),
                    }
            }),
            new BCPath(new Point[][] {
                    {
                            new Point(142-60, 81),
                            new Point(142-55,61.6),
                            new Point(142-62,61.7),
                            new Point(142-24,57.5),
                    }
            }),
            new BCPath(new Point[][] {
                    {
                            new Point(142-52.6,102.4),
                            new Point(142-51.5,71.7),
                            new Point(142-58,26.1),
                            new Point(142-55.5,34),
                            new Point(142-25,34),
                    }
            })
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
    ElapsedTime timer;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    public static boolean uk = false;
    public static double gvf_threshold = 0.5;
    public static double pid_threshold = 0.8;
    public static double shoot_angle = -135;
    public static double power = 0.8;
    int loops = -1;

    ArrayList<Point> pathPoints;

    @Override
    public void init() {
        odometry = new Odometry(hardwareMap, telemetry, start_point.x, start_point.y, 180-135);
        wheelControl = new WheelControl(hardwareMap, odometry);

        vf = new VectorField(wheelControl, odometry, uk);
        timer = new ElapsedTime();

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

                if (timer.milliseconds() >= 2000){
                    loops++;
                    vf.setPath(follow_paths[loops], 0, true);
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

        dashboard.sendTelemetryPacket(packet);
    }
}


