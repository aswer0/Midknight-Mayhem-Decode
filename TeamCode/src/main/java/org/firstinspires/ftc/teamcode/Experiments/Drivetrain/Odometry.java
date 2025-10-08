package org.firstinspires.ftc.teamcode.Experiments.Drivetrain;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.Point;
import org.opencv.core.Scalar;

@Config
public class Odometry {
    Limelight3A limelight;
    PinpointOdometry pinpoint;
    Telemetry telemetry;

    public static double offset_x = -2.375;
    public static double offset_y = 5.1875;

    public static double xyVariance = 0.1;
    public static double headingVariance = 0.1;


    Mat covariance = new Mat(3, 3, CvType.CV_64FC1, new Scalar(0));
    // x, y, yaw
    Mat xEstimate = new Mat(3, 1, CvType.CV_64FC1, new Scalar(0));


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

        xEstimate.put(0, 0, start_x/0.9144*36);
        xEstimate.put(1, 0, start_y/0.9144*36);
        xEstimate.put(2, 0, start_h/0.9144*36);

        covariance.put(0, 0, 1);
        covariance.put(1, 1, 1);
        covariance.put(2, 2, 1);
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


    public void updateMeasurements() {
        // TODO reject very out of pocket April tag measurements
        // TODO adjust the covariance matrices (MT1 stdevs seem very bad)
        // Predict
        LLResult result = limelight.getLatestResult();
        if(result == null) return;
        if(result.getBotposeTagCount() == 0) return;
        Position measurement = result.getBotpose().getPosition();
        Mat measurementM = new MatOfDouble(measurement.x, measurement.y, result.getBotpose().getOrientation().getYaw());
        double[] mStdDev = result.getStddevMt1();
        Mat measurementCov = new Mat(3, 3, CvType.CV_64FC1,new Scalar(0));
        measurementCov.put(0,0,xyVariance);
        measurementCov.put(1,1,xyVariance);
        measurementCov.put(2,2, headingVariance);


        // Calc Kalman Gain
        Mat kalmanGain = new Mat(3, 3, CvType.CV_64FC1);
        covariance.copyTo(kalmanGain);
        Core.add(measurementCov, kalmanGain, kalmanGain); // S = P + R
        kalmanGain = covariance.matMul(kalmanGain.inv()); // P*S^-1
        Mat a = new Mat(3, 3, CvType.CV_64FC1);
        Core.subtract(Mat.eye(3,3,CvType.CV_64FC1), kalmanGain,a); // I - K

        // Update Covariance
        Mat aT = new Mat(3, 3, CvType.CV_64FC1); // (I-K)^T
        Core.transpose(a, aT);
        covariance = a.matMul(covariance).matMul(aT); // (I - K)*P*(I - K)^T
        Mat kT = new Mat(3, 3, CvType.CV_64FC1);
        Core.transpose(kalmanGain, kT);
        Core.add(covariance, kalmanGain.matMul(measurementCov).matMul(kT), covariance); // (I - K)*P*(I - K)^T * KRK^t

        // Update X variable
        Mat xDelta = new Mat(3, 1, CvType.CV_64FC1); // (x - m)
        Core.subtract(measurementM, xEstimate, xDelta);
        Core.add(xEstimate, kalmanGain.matMul(xDelta), xEstimate);// x + K(x - m)
        xEstimate.put(2,0,wrapAngle(xEstimate.get(2,0)[0]));

        pinpoint.setPosition(new Pose2D(DistanceUnit.METER, xEstimate.get(0,0)[0], xEstimate.get(1,0)[0], AngleUnit.DEGREES, xEstimate.get(2,0)[0]));
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("X", xEstimate.get(0,0)[0]/0.9144*36);
        packet.put("Y", xEstimate.get(1,0)[0]/0.9144*36);
        packet.put("H", xEstimate.get(2,0)[0]);
        packet.put("Measured X", measurement.x/0.9144*36);
        packet.put("Measured Y", measurement.y/0.9144*36);
        packet.put("Measured H", result.getBotpose().getOrientation().getYaw());
        (FtcDashboard.getInstance()).sendTelemetryPacket(packet);
    }


    public void predictMeasurement(double currentTime) {
        // updateX
        // (y, -x)
        Pose2D position = pinpoint.getPosition();
        xEstimate.put(0,0,position.getX(DistanceUnit.METER));
        xEstimate.put(1,0,position.getY(DistanceUnit.METER));
        xEstimate.put(2,0,position.getHeading(AngleUnit.DEGREES));
        xEstimate.put(2,0,wrapAngle(xEstimate.get(2,0)[0]));

        //update Covariances
        Mat stdDevM = (new Mat(3, 3, CvType.CV_64FC1, new Scalar(0)));
        stdDevM.put(0,0,Math.pow(0.1, 2)); // TODO these need to be tuned
        stdDevM.put(1,1,Math.pow(0.1, 2));
        stdDevM.put(2,2,Math.pow(0.1, 2));
        //Core.multiply(stdDevM, new Scalar(currentTime - lastTime), stdDevM);
        Core.add(covariance, stdDevM, covariance);

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("X", xEstimate.get(0,0)[0]/0.9144*36);
        packet.put("Y", xEstimate.get(1,0)[0]/0.9144*36);
        packet.put("H", xEstimate.get(2,0)[0]);
        (FtcDashboard.getInstance()).sendTelemetryPacket(packet);
    }

    public double wrapAngle(double heading) {
        heading += 180;
        while (heading < 0) heading += 360;
        return (heading) % 360 - 180;
    }
}
