package org.firstinspires.ftc.teamcode.Experiments.Drivetrain;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.opencv.core.Point;

@Config
public class Odometry {
    PinpointOdometry pinpoint;
    Telemetry telemetry;

    public static double offset_x = -2.375;
    public static double offset_y = 5.1875;

    double start_x;
    double start_y;
    double start_h;

    public Odometry(HardwareMap hardwareMap, Telemetry telemetry, String pinpointName, double start_x, double start_y, double start_h){
        this.pinpoint = hardwareMap.get(PinpointOdometry.class, pinpointName);
        this.telemetry = telemetry;

        pinpoint.setEncoderResolution(PinpointOdometry.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(PinpointOdometry.EncoderDirection.REVERSED, PinpointOdometry.EncoderDirection.FORWARD);
        /*
        Horizontal: 2.375in
        Vertical: 5.1875in
        * */

        this.start_x = start_x;
        this.start_y = start_y;
        this.start_h = start_h;

        pinpoint.setOffsets(offset_x, offset_y, DistanceUnit.INCH);
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, start_x, start_y, AngleUnit.DEGREES, start_h));
        pinpoint.recalibrateIMU();
    }

    public void update(){
        pinpoint.update();
    }
    public double get_turret_heading(){
        Point target = new Point(125, 130);

        double angleToTarget = Math.toDegrees(Math.atan2(target.y - this.get_y(), target.x - this.get_x()));
        double turretAngle = angleToTarget - this.get_heading();

        while (turretAngle > 180) turretAngle -= 360;
        while (turretAngle < -180) turretAngle += 360;

        return turretAngle;

    }
    public void reset_original_pos(){
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, start_x, start_y, AngleUnit.DEGREES, start_h));
        pinpoint.recalibrateIMU();
    }
    public double get_heading(){
        return pinpoint.getPosition().getHeading(AngleUnit.DEGREES);
    }
    public double get_x(){
        return pinpoint.getPosX(DistanceUnit.INCH);
    }
    public double get_y(){
        return pinpoint.getPosY(DistanceUnit.INCH);
    }
}
