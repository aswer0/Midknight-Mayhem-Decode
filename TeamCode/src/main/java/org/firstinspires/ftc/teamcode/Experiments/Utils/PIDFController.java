package org.firstinspires.ftc.teamcode.Experiments.Utils;

import com.qualcomm.robotcore.util.ElapsedTime;


public class PIDFController {
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

    public PIDFController(double kp, double ki, double kd,double kf) {
        this.kp = kp;
        this.kd = kd;
        this.ki = ki;
        this.kf = kf;
        timer = new ElapsedTime();
    }

    public double calculate(double tar, double act) {
        double s = timer.seconds();
        e = tar - act;
        p = kp * (e);
        i = ki * iSum;
        d = kd * ((e - e_last) / s);

        iSum += s * e;
        e_last = e;
        timer.reset();
        return p + i + d + kf;
    }
}
