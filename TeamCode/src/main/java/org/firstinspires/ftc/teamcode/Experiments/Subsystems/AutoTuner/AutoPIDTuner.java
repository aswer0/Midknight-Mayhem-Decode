package org.firstinspires.ftc.teamcode.Experiments.Subsystems.AutoTuner;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Drivetrain.WheelControl;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.TreeMap;

@TeleOp
@Config
public class AutoPIDTuner extends OpMode {

    public static double eta_p = 1, eta_d = 1;
    public static boolean uk = false;

    double tx=0, ty=0, th=0;
    public static ArrayList<Vec3d> target_points;
    int batch = 0;

    Odometry odometry;
    WheelControl wheelControl;
    ElapsedTime timer;

    public static KGains gains;

    public static Vec3d t = new Vec3d();

    public static TreeMap<Double, Vec3d> errors;
    public static Vec3d t_finals = new Vec3d();
    public static Vec3d t_starts = new Vec3d();

    boolean overshoot_x = false;
    boolean overshoot_y = false;
    boolean overshoot_h = false;

    enum State{
        compute,
        update,
        reset,
        computeReset,
        fullReset
    }

    State state = State.compute;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void init() {
        odometry = new Odometry(hardwareMap, telemetry, 7.875, 6.625, 0);
        wheelControl = new WheelControl(hardwareMap, odometry);
        timer = new ElapsedTime();

        target_points.add(new Vec3d(30, 30, 90));
        target_points.add(new Vec3d(20, 52.5, 120));
        target_points.add(new Vec3d(60, 81, 135));
        target_points.add(new Vec3d(40, 45, 45));

        gains = new KGains(
                telemetry,
                new Vec3d(0.3, 0.3, 0.3),
                new Vec3d(0.01, 0.01, 0.01),
                eta_p, eta_d
        );
    }

    public boolean compute(){
        if (batch != -1){
            tx = target_points.get(batch).x;
            ty = target_points.get(batch).y;
            th = target_points.get(batch).h;
        }
        else{
            tx = 0;
            ty = 0;
            th = 0;
        }

        wheelControl.drive_to_point(new Point(tx, ty), th, 1, 0.5, false);

        Vec3d r = new Vec3d(odometry.get_x(uk), odometry.get_y(uk), odometry.get_heading(uk));

        Vec3d e = new Vec3d(t);
        e.subtract(r);

        errors.put(timer.milliseconds(), e);

        boolean reached_x=false, reached_y=false, reached_h=false;
        if (odometry.get_x_velocity() <= 0.2){
            t_finals.x = timer.milliseconds();
            reached_x = true;
        }
        if (odometry.get_y_velocity() <= 0.2){
            t_finals.y = timer.milliseconds();
            reached_y = true;
        }
        if (odometry.get_h_velocity() <= 0.2){
            t_finals.h = timer.milliseconds();
            reached_h = true;
        }

        if (!overshoot_x && e.x < 0){
            t_starts.x = timer.milliseconds();
            overshoot_x = true;
        }
        if (!overshoot_y && e.y < 0){
            t_starts.y = timer.milliseconds();
            overshoot_y = true;
        }
        if (!overshoot_h && e.h < 0){
            t_starts.h = timer.milliseconds();
            overshoot_h = true;
        }

        return reached_x && reached_y && reached_h;
    }
    public double integrate(ArrayList<Point> points){
        double A = 0;
        int n = points.size();

        for (int i=0; i<n-1; i++){
            double h = points.get(i+1).x - points.get(i).x;
            A += (h*(points.get(i).y + points.get(i+1).y))/2;
        }

        return A;
    }

    public double compute_oscillation(double t_start, double t_end, char axis){
        Map<Double, Vec3d> pts_to_integrate = errors.subMap(t_start, true, t_end, true);
        ArrayList<Point> points = new ArrayList<>();

        for (Map.Entry<Double, Vec3d> pts : pts_to_integrate.entrySet()) {
            points.add(new Point(pts.getKey(), pts.getValue().get(axis)));
        }

        return integrate(points);

    }

    public void update_(){
        Vec3d E = new Vec3d(errors.get(t_finals.x).x, errors.get(t_finals.y).y, errors.get(t_finals.h).h);

        Vec3d O = new Vec3d();
        O.x = compute_oscillation(t_starts.x, t_finals.x, 'x');
        O.y = compute_oscillation(t_starts.y, t_finals.y, 'y');
        O.h = compute_oscillation(t_starts.h, t_finals.h, 'h');

        gains.Update(E, O);
        wheelControl.update_gains(gains.get_p_gains(), new Vec3d(0, 0, 0), gains.get_d_gains());
    }

    @Override
    public void loop() {
        odometry.update();

        switch (state){
            case compute:
                batch = (int)(Math.random() * (target_points.size()));
                if (compute()){
                    update_();
                    state = State.reset;
                }
                break;

            case reset:
                timer.reset();
                state = State.computeReset;
                break;

            case computeReset:
                batch = -1;
                if (compute()){
                    update_();
                    timer.reset();
                    state = State.fullReset;
                }

            case fullReset:
                if (timer.milliseconds() <= 1000){
                    wheelControl.drive(-1, 0, 0, 0, 0.5);
                }
                else if (timer.milliseconds() <= 2000){
                    wheelControl.drive(0, 1, 0, 0, 0.5);
                }
                else{
                    timer.reset();
                    state = State.compute;
                }
        }

        gains.debug();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("error", errors.toString());
        packet.put("t finals", t_finals.toString());
        packet.put("t starts", t_starts.toString());
        packet.put("p gains", gains.get_p_gains());
        packet.put("d gains", gains.get_d_gains());

    }
}
