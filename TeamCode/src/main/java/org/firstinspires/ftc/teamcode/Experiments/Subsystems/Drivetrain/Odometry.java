package org.firstinspires.ftc.teamcode.Experiments.Subsystems.Drivetrain;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.Point;
import org.opencv.core.Scalar;


// ToDO: Scale cov with measured position
@Config
public class Odometry {
    final String pinpointName = "pinpoint";
    final String limelightName = "limelight";
    Limelight3A limelight;
    PinpointOdometry pinpoint;
    Servo llServo;
    Telemetry telemetry;
    ElapsedTime lastMeasurementTime;

    public static double offset_x = -2.375;
    public static double offset_y = 5.1875;

    public static double xyVariance = 25;
    public static double headingVariance = 100;

    public static double llPosition = 0.88;

    public static boolean outputDebugInfo = true;



    Mat covariance = new Mat(3, 3, CvType.CV_64FC1, new Scalar(0));
    // xMeasurements, yMeasurements, yaw
    Mat xEstimate = new Mat(3, 1, CvType.CV_64FC1, new Scalar(0));

    double start_x;
    double start_y;
    double start_h;

    public Odometry(HardwareMap hardwareMap, Telemetry telemetry, double start_x, double start_y, double start_h){
        limelight = hardwareMap.get(Limelight3A.class, limelightName);
        this.pinpoint = hardwareMap.get(PinpointOdometry.class, pinpointName);
        this.telemetry = telemetry;
        llServo = hardwareMap.get(Servo.class, "llServo");
        llServo.setPosition(llPosition);
        limelight.pipelineSwitch(0);
        limelight.start();

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
        llServo.setPosition(llPosition);
        pinpoint.update();
        predictMeasurement();
        //updateMeasurements();
    }
    public double get_turret_heading(boolean use_kalman){
        Point target = new Point(125, 130);

        double angleToTarget = Math.toDegrees(Math.atan2(target.y - this.get_y(use_kalman), target.x - this.get_x(use_kalman)));
        double turretAngle = angleToTarget - this.get_heading(use_kalman);

        while (turretAngle > 180) turretAngle -= 360;
        while (turretAngle < -180) turretAngle += 360;

        return turretAngle;

    }
    public void reset_original_pos(){
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, start_x, start_y, AngleUnit.DEGREES, start_h));
        pinpoint.recalibrateIMU();
    }
    public double get_heading(boolean use_kalman){
        if (use_kalman){
            return xEstimate.get(2, 0)[0];
        }
        return pinpoint.getPosition().getHeading(AngleUnit.DEGREES);
    }
    public double get_x(boolean use_kalman){
        if (use_kalman){
            return xEstimate.get(0, 0)[0] / 0.9144 * 36;
        }
        return pinpoint.getPosX(DistanceUnit.INCH);
    }
    public double get_y(boolean use_kalman){
        if (use_kalman){
            return xEstimate.get(1, 0)[0] / 0.9144 * 36;
        }
        return pinpoint.getPosY(DistanceUnit.INCH);
    }

    // the math is based on https://file.tavsys.net/control/controls-engineering-in-frc.pdf#page=177
    public void updateMeasurements(LLResultTypes.FiducialResult tag) {

        Pose3D measurement = tag.getRobotPoseFieldSpace();
        // reject too deviant measurements
        //if(Math.pow(measurement.xMeasurements - xEstimate.get(0,0)[0],2) + Math.pow(measurement.yMeasurements - xEstimate.get(1,0)[0], 2) > Math.pow(0.25,2)) return;
        Mat measurementM = new MatOfDouble(measurement.getPosition().x, measurement.getPosition().y, measurement.getOrientation().getYaw());
        Mat measurementCov = calculateMeasurementCovariance(tag);

        // Calc Kalman Gain
        // K = PCᵀS⁻¹ where S = (CPCᵀ + R) = P + R (because C = I)
        // KS = PCᵀ
        // SᵀKᵀ = CPᵀ
        Mat kT = new Mat(3, 3, CvType.CV_64FC1); // kT = solve(Sᵀ, CPᵀ or CP)
        Core.add(covariance, measurementCov, kT);
        Core.solve(kT, covariance, kT);
        Mat kalmanGain = new Mat(3, 3, CvType.CV_64FC1);
        Core.transpose(kT, kalmanGain);

        // Update Covariance
        Mat a = new Mat(3, 3, CvType.CV_64FC1);
        Core.subtract(Mat.eye(3,3,CvType.CV_64FC1), kalmanGain,a);
        Mat aT = new Mat(3, 3, CvType.CV_64FC1); // (I-K)^T
        Core.transpose(a, aT);
        covariance = a.matMul(covariance).matMul(aT); // (I - K)*P*(I - K)^T
        Core.add(covariance, kalmanGain.matMul(measurementCov).matMul(kT), covariance); // (I - K)*P*(I - K)^T + KRK^t

        // Update X variable
        Mat xDelta = new Mat(3, 1, CvType.CV_64FC1); // (xMeasurements - m)
        Core.subtract(measurementM, xEstimate, xDelta);
        Core.add(xEstimate, kalmanGain.matMul(xDelta), xEstimate);// xMeasurements + K(xMeasurements - m)
        xEstimate.put(2,0,wrapAngle(xEstimate.get(2,0)[0]));

        pinpoint.setPosition(new Pose2D(DistanceUnit.METER, xEstimate.get(0,0)[0], xEstimate.get(1,0)[0], AngleUnit.DEGREES, xEstimate.get(2,0)[0]));
        if(outputDebugInfo) {
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("X", xEstimate.get(0, 0)[0] / 0.9144 * 36);
            packet.put("Y", xEstimate.get(1, 0)[0] / 0.9144 * 36);
            packet.put("H", xEstimate.get(2, 0)[0]);
            packet.put("Measured X", measurement.getPosition().x / 0.9144 * 36);
            packet.put("Measured Y", measurement.getPosition().y / 0.9144 * 36);
            packet.put("Measured H", measurement.getOrientation().getYaw());
            packet.fieldOverlay().fillRect(xEstimate.get(0,0)[0] / 0.9144 * 36, xEstimate.get(1,0)[0] / 0.9144 * 36, 2, 2);
            (FtcDashboard.getInstance()).sendTelemetryPacket(packet);
        }
    }

    public Mat calculateMeasurementCovariance(LLResultTypes.FiducialResult tag) {
        Mat measurementCov = new Mat(3, 3, CvType.CV_64FC1,new Scalar(0));
        measurementCov.put(0,0,xyVariance);
        measurementCov.put(1,1,xyVariance);
        measurementCov.put(2,2, headingVariance);
        return measurementCov;
    }
    public void predictMeasurement() {
        // updateX
        // (yMeasurements, -xMeasurements)
        Pose2D position = pinpoint.getPosition();
        xEstimate.put(0,0,position.getX(DistanceUnit.METER));
        xEstimate.put(1,0,position.getY(DistanceUnit.METER));
        xEstimate.put(2,0,position.getHeading(AngleUnit.DEGREES));
        xEstimate.put(2,0,wrapAngle(xEstimate.get(2,0)[0]));
        limelight.updateRobotOrientation(position.getHeading(AngleUnit.DEGREES));

        //update Covariances
        Mat stdDevM = (new Mat(3, 3, CvType.CV_64FC1, new Scalar(0)));
        stdDevM.put(0,0,Math.pow(0.1, 2));
        stdDevM.put(1,1,Math.pow(0.1, 2)); // TODO: scale covariance with displacement
        stdDevM.put(2,2,Math.pow(0.1, 2));
        //Core.multiply(stdDevM, new Scalar(currentTime - lastTime), stdDevM);
        Core.add(covariance, stdDevM, covariance);

        if(outputDebugInfo) {
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("X", xEstimate.get(0, 0)[0] / 0.9144 * 36);
            packet.put("Y", xEstimate.get(1, 0)[0] / 0.9144 * 36);
            packet.put("H", xEstimate.get(2, 0)[0]);
            packet.fieldOverlay().fillRect(xEstimate.get(0,0)[0] / 0.9144 * 36, xEstimate.get(1,0)[0] / 0.9144 * 36, 2, 2);
            (FtcDashboard.getInstance()).sendTelemetryPacket(packet);
        }
    }

    public double wrapAngle(double heading) {
        heading += 180;
        while (heading < 0) heading += 360;
        return (heading) % 360 - 180;
    }
}
