package org.firstinspires.ftc.teamcode.Experiments.Drivetrain;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDFController;

@Config
public class DriveCorrection {
    public static double stable_hp = 0.00013, stable_hi = 0, stable_hd = 0.001;
    public static double set_hp = 0.1, set_hi = 0, set_hd = 0.001;
    public static double tolerance = 0.15;

    Odometry odometry;

    PIDFController stable_correction;
    PIDFController set_correction;

    public DriveCorrection(Odometry odometry){
        this.odometry = odometry;

        stable_correction = new PIDFController(stable_hp, stable_hi, stable_hd, 0);
        set_correction = new PIDFController(set_hd, set_hi, set_hp, 0);
    }

    public double stable_correction(double target_angle, boolean use_kalman){
        double error = stable_correction.calculate(odometry.get_heading(use_kalman), target_angle);
        if (error <= tolerance){
            return 0.0;
        }
        return error;
    }

    public double set_correction(double target_angle, boolean use_kalman){
        return set_correction.calculate(odometry.get_heading(use_kalman), target_angle);
    }
}
