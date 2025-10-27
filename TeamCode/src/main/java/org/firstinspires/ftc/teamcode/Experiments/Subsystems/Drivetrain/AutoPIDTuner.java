package org.firstinspires.ftc.teamcode.Experiments.Subsystems.Drivetrain;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Core;
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

    public static double tx=30, ty=30, th=90;

    Odometry odometry;
    WheelControl wheelControl;

    public static Mat k_matrix = new Mat(3, 2, CvType.CV_64FC1);
    public static Mat t_matrix = new Mat(3, 1, CvType.CV_64FC1);

    @Override
    public void init() {
        odometry = new Odometry(hardwareMap, telemetry, 7.875, 6.625, 0);
        wheelControl = new WheelControl(hardwareMap, odometry);

        k_matrix.col(0).setTo(new Scalar(0.3));
        k_matrix.col(1).setTo(new Scalar(0.02));

        t_matrix.put(0, 0, tx);
        t_matrix.put(0, 1, ty);
        t_matrix.put(0, 2, th);
    }

    public double s(double x, double k) {
        return Math.tanh(x*k);
    }

    @Override
    public void loop() {
        odometry.update();

        Mat r_matrix = new Mat(3, 1, CvType.CV_64FC1);
        r_matrix.put(0, 0, odometry.get_x(uk));
        r_matrix.put(0, 1, odometry.get_y(uk));
        r_matrix.put(0, 2, odometry.get_heading(uk));

        Mat kp = new Mat(3, 1, CvType.CV_64FC1);
        Mat kd = new Mat(3, 1, CvType.CV_64FC1);
        Core.subtract(r_matrix, t_matrix, kp);
        Core.subtract(t_matrix, r_matrix, kd);

        Mat grad_matrix = new Mat(3, 2, CvType.CV_64FC1);

    }
}
