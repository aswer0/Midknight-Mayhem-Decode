package org.firstinspires.ftc.teamcode.Experiments.Utils;

import com.qualcomm.robotcore.util.ElapsedTime;


public class PIDFController {
    private double kp;
    private double kd;
    private double kf;
    private double p;
    private double d;
    private double e;
    private double e_last;
    private ElapsedTime timer;

    private PIDFController(double kp, double kd) {
        this.kp = kp;
        this.kd = kd;
        this.kf = kf;
        timer = new ElapsedTime();
    }

    public double Calculate(double tar, double act) {
        e = tar - act;
        p = kp * (e);
        d = kd * ((e - e_last) / timer.seconds());
        e_last = e;
        timer.reset();
        return p + d + kf;
    }
}
