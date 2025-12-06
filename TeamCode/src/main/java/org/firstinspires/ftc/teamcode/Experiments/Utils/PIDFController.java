package org.firstinspires.ftc.teamcode.Experiments.Utils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config
public class PIDFController {
    public static double iClamp = 3;
    private double kp;
    private double kd;
    private double ki;
    private double kf;
    private double p;
    private double i;
    private double d;
    private double e;
    private double e_last;
    private double iSum = 0;
    private ElapsedTime timer;

    public PIDFCoefficients gains;

    public PIDFController(double kp, double ki, double kd,double kf) {
        this.kp = kp;
        this.kd = kd;
        this.ki = ki;
        this.kf = kf;
        timer = new ElapsedTime();
        gains = new PIDFCoefficients(kp, ki, kd, kf);
    }
    public PIDFController(PIDFCoefficients coefficients) {
        this.kp = coefficients.p;
        this.kd = coefficients.d;
        this.ki = coefficients.i;
        this.kf = coefficients.f;
        timer = new ElapsedTime();
        gains = coefficients;
    }

    public static double wrapError(double target, double current) {
        double error = (target - current + 180) % 360;
        if (error < 0) error += 360;
        return error - 180;
    }
    public double calculate_heading(double tar, double act, double fOverride) {
//        double s = timer.seconds();
//
//        e = wrapError(tar, act);
//        p = gains.p * (e);
//        i = gains.i * iSum;
//        d = gains.d * ((e - e_last) / s);
//
//        iSum += s * e;
//        e_last = e;
//        timer.reset();
//        return p + d + gains.f;

        double s = timer.seconds();
        e = wrapError(tar, act);
        p = gains.p * (e);
        i = gains.i * iSum;
        d = gains.d * ((e - e_last) / s);

        iSum = Math.max(Math.min(iSum + s * e, iClamp),-iClamp);
        e_last = e;
        timer.reset();
        return p + i + d + fOverride;
    }
    public double calculate_heading(double tar, double act) {
        return calculate_heading(tar, act, gains.f);
    }

    public double calculate(double tar, double act, double fOverride) {
        double s = timer.seconds();
        e = tar - act;
        p = gains.p * (e);
        i = gains.i * iSum;
        d = gains.d * ((e - e_last) / s);

        iSum = Math.max(Math.min(iSum + s * e, iClamp),-iClamp);
        e_last = e;
        timer.reset();
        return p + i + d + fOverride;
    }
    public double calculate(double tar, double act) {
        return calculate(tar, act, gains.f);
    }
}
