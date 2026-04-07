package org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Outtake;

import java.util.Arrays;
import java.util.List;

public class Model {
    private static List<List<Double> > coeffs = Arrays.asList(
        Arrays.asList(0.0275028, 0.362241, -0.69396, 0.0686839),
        Arrays.asList(0.0770587, -6.26647, 124.0487),
        Arrays.asList(-3.11963, 262.7517, -3158.52948)
    );
    //
    public static double rpm_curr;
    public static double auto_rpm;
    public static double auto_hood;

    public static double F(double dist, double angle){
        if (dist >= 142){
            return 3332;
        }

        List<Double> A = coeffs.get(0);
        List<Double> B = coeffs.get(1);
        List<Double> C = coeffs.get(2);
        double a_coeff = A.get(0)*Math.sin(A.get(1)*angle + A.get(2)) + A.get(3);
        double b_coeff = B.get(0)*angle*angle + B.get(1)*angle + B.get(2);
        double c_coeff = C.get(0)*angle*angle + C.get(1)*angle + C.get(2);

        return a_coeff*dist*dist + b_coeff*dist + c_coeff;
    }

    public static void update_values(double dist){
        double angle = dF_dtheta_descent(dist);
        auto_rpm = F(dist, angle);
        auto_hood = angle;
    }

    public static double dF_dtheta(double theta, double dist){
        double h = 1e-5;
        return (F(dist, theta+h) - F(dist, theta-h))/(2*h);
    }

    public static double dF_dtheta_descent(double dist){
        double dH_dtheta;
        double theta_new;

        double theta_old = 40;
        double lr = 0.06;
        for (int i=0; i<7; i++){
            dH_dtheta = 2*(F(dist, theta_old) - rpm_curr) * dF_dtheta(theta_old, dist);
            theta_new = theta_old - lr*dH_dtheta;
            theta_old = theta_new;
        }

        return theta_old;
    }
}
