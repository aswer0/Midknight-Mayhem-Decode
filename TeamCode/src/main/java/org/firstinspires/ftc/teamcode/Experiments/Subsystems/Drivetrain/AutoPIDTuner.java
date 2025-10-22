package org.firstinspires.ftc.teamcode.Experiments.Subsystems.Drivetrain;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;

@TeleOp
@Config
public class AutoPIDTuner extends OpMode {
    public static double xp = 0.25, xi = 0, xd = 0.01, xf = 0;
    public static double yp = 0.3, yi = 0, yd = 0.008, yf = 0;
    public static double hp = 0.03, hi = 0, hd = 0.003, hF = 0;

    public static double eta_p = 1, eta_d = 1;
    public static double k = 0.5;
    public static boolean uk = false;

    public static Point t = new Point(30, 30);
    public static double ta = 90;

    Odometry odometry;
    WheelControl wheelControl;

    public static Mat gains = new Mat(3, 2, CvType.CV_64FC1, new Scalar(0.2));

    @Override
    public void init() {
        odometry = new Odometry(hardwareMap, telemetry, 7.875, 6.625, 0);
        wheelControl = new WheelControl(hardwareMap, odometry);

        gains.row(0).setTo(new Scalar(0.3));
        gains.row(1).setTo(new Scalar(0.02));
    }

    public double s(double x, double k) {
        return Math.tanh(x*k);
    }

    @Override
    public void loop() {
        odometry.update();

        Mat pos = new Mat(1, 3, CvType.CV_64FC1);
        pos.put(0, 0, odometry.get_x(uk));
        pos.put(0, 1, odometry.get_y(uk));
        pos.put(0, 2, odometry.get_heading(uk));

        Mat grad = new Mat(2, 3, CvType.CV_64FC1);

        grad.put(0, 0, pos.get(0, 0)[0]-t.x);
        grad.put(0, 1, pos.get(0, 1)[0]-t.y);
        grad.put(0, 2, pos.get(0, 1)[0]-ta);

        grad.put(1, 0, t.x-pos.get(0, 0)[0]);
        grad.put(1, 1, t.y-pos.get(0, 1)[0]);
        grad.put(1, 2, ta-pos.get(0, 1)[0]);



    }
}
