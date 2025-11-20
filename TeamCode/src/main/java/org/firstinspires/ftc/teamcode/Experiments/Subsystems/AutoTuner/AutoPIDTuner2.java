package org.firstinspires.ftc.teamcode.Experiments.Subsystems.AutoTuner;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Drivetrain.GVF.BCPath;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Drivetrain.GVF.VectorField;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Drivetrain.WheelControl;
import org.opencv.core.Point;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.TreeMap;

@TeleOp
@Config
public class AutoPIDTuner2 extends OpMode{
    public static double xp = 0.2, xi = 0, xd = 0;
    public static double yp = 0.2, yi = 0, yd = 0;
    public static double hp = 0.2, hi = 0, hd = 0;

    public static boolean uk = false;

    public static Vec3d ultimate;
    public static Vec3d period;

    double tx=50, ty=50, th=90;

    Odometry odometry;
    WheelControl wheelControl;
    ZieglerNichols tuner;
    ElapsedTime timer;

    enum State{
        compute,
        reset,
        align,
        update
    }

    State state = State.compute;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void init() {
        odometry = new Odometry(hardwareMap, telemetry, 8, 8, 0);
        wheelControl = new WheelControl(hardwareMap, odometry);
        tuner = new ZieglerNichols();

        wheelControl.update_gains(
                new Vec3d(xp, xi, xd),
                new Vec3d(yp, yi, yd),
                new Vec3d(hp, hi, hd)
        );
    }
    @Override
    public void loop(){
        odometry.update();

        TelemetryPacket packet = new TelemetryPacket();

        switch (state){
            case compute:
                wheelControl.drive_to_point(new Point(tx, ty), th, 1, 0.5, uk);
                break;

            case reset:
                wheelControl.original_gains();
                if (wheelControl.drive_to_point(new Point(0, 0), 0, 0.8, 0.5, uk)){
                    state = State.align;
                }
                break;

            case align:
                wheelControl.drive(-1, 1, 0, 0, 0.5);
                odometry.set_heading(0);
                odometry.set_x(8);
                odometry.set_y(8);
                break;

            case update:
                tuner.set_params(ultimate.x, period.x);
                Vec3d x_gains = tuner.Update();
                tuner.set_params(ultimate.y, period.y);
                Vec3d y_gains = tuner.Update();
                tuner.set_params(ultimate.h, period.h);
                Vec3d h_gains = tuner.Update();

                wheelControl.update_gains(
                        x_gains,
                        y_gains,
                        h_gains
                );

        }


    }

}
