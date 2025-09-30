package org.firstinspires.ftc.teamcode.Experiments.Utils;

import com.qualcomm.robotcore.util.ElapsedTime;


public class PIDFController {
    private double kp;
    private double kd;
    private double ki;
    private double kf;
    private double p;
    private double d;
    private double e;
    private double e_last;
    private ElapsedTime timer;

    public PIDFController(double kp, double ki, double kd,double kf) {
        this.kp = kp;
        this.kd = kd;
        this.ki = 0;
        this.kf = kf;
        timer = new ElapsedTime();
    }

    public double calculate(double tar, double act) {
        e = tar - act;
        p = kp * (e);
        d = kd * ((e - e_last) / timer.seconds());
        e_last = e;
        timer.reset();
        return p + d + kf;
    }
}
