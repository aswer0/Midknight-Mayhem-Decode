package org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Outtake;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;
import java.util.List;

@Config
public class Model {
    public static double last_coeff = -3158.52948;

    private static List<List<Double> > coeffs = Arrays.asList(
        Arrays.asList(0.0275028, 0.362241, -0.69396, 0.0686839),
        Arrays.asList(0.0770587, -6.26647, 124.0487),
        Arrays.asList(-3.11963, 262.7517, last_coeff)
    );
    //0.0275028\cdot\sin\left(20.75486x-0.69396\right)+0.0686839
    public static double rpm_curr;
    public static double auto_rpm;
    public static double auto_hood;
    public static FtcDashboard dashboard = FtcDashboard.getInstance();

    public static int EPOCHS = 15;
    public static int MIN_DIST = 70;
    public static double LR = 1e-3;

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
        double h = 1e-3;
        return (F(dist, theta+h) - F(dist, theta-h))/(2*h);
    }

//    public static double dF_dtheta_descent(double dist){
//        double dH_dtheta;
//        double theta_new;
//
//        double theta_old = 40;
//        TelemetryPacket packet = new TelemetryPacket();
//        for (int i=0; i<EPOCHS; i++){
//            dH_dtheta = 2*(F(dist, theta_old) - rpm_curr) * dF_dtheta(theta_old, dist);
//            theta_new = theta_old - LR*dH_dtheta;
//            theta_old = theta_new;
////            packet.put("MODEL df_dtheta angle step " + i, theta_old);
//        }
//        dashboard.sendTelemetryPacket(packet);
////        return 45;
//        return theta_old;
//    }
    /*
    def dF_dtheta_descent(dist, lambda_=0):
    global _min_cost_changes

    theta_min = 30
    min_cost = float("inf")

    if dist >= 80:
        min_angle = 40
        max_angle = 50
    else:
        min_angle = 30
        max_angle = 38

    for theta in range(min_angle, max_angle+1, 1):
        rpm = F(dist, theta)
        cost = lambda_*rpm + (1-lambda_)*abs(rpm - rpm_curr)
        if cost < min_cost:
            min_cost = cost
            theta_min = theta

    _min_cost_changes = min_cost
    return theta_min
    * */
    public static double dF_dtheta_descent(double dist){
        double theta_min = 30;
        double min_cost = 100000000;
        int min_angle, max_angle;

        if (dist >= MIN_DIST) {
            min_angle = 40; max_angle = 50;
        }
        else{
            min_angle = 30; max_angle = 38;
        }

        for (int theta=min_angle; theta<max_angle+1; theta++){
            double rpm = F(dist, theta);
            double cost = Math.abs(rpm - rpm_curr);
            if (cost < min_cost){
                min_cost = cost;
                theta_min = theta;
            }
        }

        return theta_min;

    }
}
