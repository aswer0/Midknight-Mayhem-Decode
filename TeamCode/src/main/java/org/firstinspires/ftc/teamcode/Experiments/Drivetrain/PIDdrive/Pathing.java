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
    int count = 0;

    public Pathing(Odometry odometry, WheelControl wheelControl) {
        this.odometry = odometry;
        this.wheelControl = wheelControl;
    }

    public boolean pointDriver(ArrayList<Point> points, double end_heading, boolean use_kalman) {
        double last_point = points.size() - 1;

        double rotation = Math.atan2(points.get(count).y - odometry.get_y(use_kalman), points.get(count).x - odometry.get_x(use_kalman));

        if (count == last_point) {
            if (wheelControl.drive_to_point(points.get(count), end_heading, 0.5, 0.5, use_kalman)){
                return true;
            }
        } else if (wheelControl.drive_to_point(points.get(count), rotation, 0.5, 0.5, use_kalman)){
            count++;
        }
        return false;
    }
}
