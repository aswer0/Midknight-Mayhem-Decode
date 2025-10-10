package org.firstinspires.ftc.teamcode.Experiments.Drivetrain.PIDdrive;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.opencv.core.Point;
import java.util.ArrayList;

@Config
public class Pathing {
    Odometry odometry;
    WheelControl wheelControl;
    ArrayList<Point> path;
    
    int current_point = 0;

    public Pathing(Odometry odometry, WheelControl wheelControl, ArrayList<Point> init_path) {
        this.odometry = odometry;
        this.wheelControl = wheelControl;
        this.path = init_path;
    }

    public Pathing(Odometry odometry, WheelControl wheelControl) {
        this.odometry = odometry;
        this.wheelControl = wheelControl;
    }

    public void set_path(ArrayList<Point> new_path){
        path = new_path;
    }

    public boolean drive_to_point(Point p, double power, double dist_tresh, boolean use_kalman){
        double rotation = Math.atan2(p.y - odometry.get_y(use_kalman), p.x - odometry.get_x(use_kalman));
        return wheelControl.drive_to_point(p, rotation, power, dist_tresh, use_kalman);
    }
    public boolean drive_to_point(Point p, double power, double targete_h, double dist_tresh, boolean use_kalman){
        return wheelControl.drive_to_point(p, targete_h, power, dist_tresh, use_kalman);
    }

    public boolean pointDriver(double end_heading, double power, double dist_tresh, int stop_point, boolean use_kalman) {
        if (stop_point == -1){
            stop_point = path.size() - 1;
        }

        double rotation = Math.atan2(path.get(current_point).y - odometry.get_y(use_kalman), path.get(current_point).x - odometry.get_x(use_kalman));

        if (current_point == stop_point) {
            if (wheelControl.drive_to_point(path.get(current_point), end_heading, power, dist_tresh, use_kalman)){
                return true;
            }
        } else if (wheelControl.drive_to_point(path.get(current_point), rotation, power, dist_tresh, use_kalman)){
            current_point++;
        }
        return false;
    }

    public boolean pointDriver(double power, double dist_tresh, int stop_point, boolean use_kalman) {
        if (stop_point == -1){
            stop_point = path.size() - 1;
        }

        double rotation = Math.atan2(path.get(current_point).y - odometry.get_y(use_kalman), path.get(current_point).x - odometry.get_x(use_kalman));

        if (wheelControl.drive_to_point(path.get(current_point), rotation, power, dist_tresh, use_kalman)){
            if (current_point == stop_point) {
                return true;
            }
            current_point++;
        }
        return false;
    }

    public Point get_current_point(){
        return path.get(current_point);
    }
}
