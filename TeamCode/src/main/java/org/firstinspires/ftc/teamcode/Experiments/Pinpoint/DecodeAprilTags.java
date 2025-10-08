package org.firstinspires.ftc.teamcode.Experiments.Pinpoint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfFloat;
import org.opencv.core.Scalar;

@Config
@TeleOp
public class DecodeAprilTags extends OpMode {
    Limelight3A limelight;
    SparkFunOTOS otos;
    double lastTime = 0;

    public static double xyVariance = 0.5;
    public static double headingVariane = 100;
    // x, y, yaw
    Mat covariance = new Mat(3, 3, CvType.CV_64FC1, new Scalar(0));
    // x, y, yaw
    Mat xEstimate = new Mat(3, 1, CvType.CV_64FC1, new Scalar(0));
    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        otos = hardwareMap.get(SparkFunOTOS.class, "OTOS");
        otos.setLinearUnit(DistanceUnit.METER);
        otos.setAngularUnit(AngleUnit.DEGREES);
        xEstimate.put(0, 0, 1);
        xEstimate.put(1, 0, 1);
        xEstimate.put(2, 0, 1);

        covariance.put(0, 0, 2);
        covariance.put(1, 1, 2);
        covariance.put(2, 2, 2);
    }

    @Override
    public void loop() {
        double currentTime = getRuntime();
        // Predict
        predictMeasurement(currentTime);

        updateMeasurements();



        lastTime = currentTime;
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
        measurementCov.put(2,2,headingVariane);

        Mat kalmanGain = new Mat(3, 3, CvType.CV_64FC1);
        covariance.copyTo(kalmanGain);
        Core.add(measurementCov, kalmanGain, kalmanGain); // S = P + R
        kalmanGain = covariance.matMul(kalmanGain.inv()); // P*S^-1
        Mat a = new Mat(3, 3, CvType.CV_64FC1);
        Core.subtract(Mat.eye(3,3,CvType.CV_64FC1), kalmanGain,a); // I - K

        Mat aT = new Mat(3, 3, CvType.CV_64FC1); // (I-K)^T
        Core.transpose(a, aT);
        covariance = a.matMul(covariance).matMul(aT); // (I - K)*P*(I - K)^T
        Mat kT = new Mat(3, 3, CvType.CV_64FC1);
        Core.transpose(kalmanGain, kT);
        Core.add(covariance, kalmanGain.matMul(measurementCov).matMul(kT), covariance); // (I - K)*P*(I - K)^T * KRK^t
        Mat xDelta = new Mat(3, 1, CvType.CV_64FC1); // (x - m)
        Core.subtract(measurementM, xEstimate, xDelta);
        Core.add(xEstimate, kalmanGain.matMul(xDelta), xEstimate);// x + K(x - m)
        xEstimate.put(2,0,wrapAngle(xEstimate.get(2,0)[0]));

        otos.setPosition(new Pose2D(xEstimate.get(0,0)[0], xEstimate.get(1,0)[0], xEstimate.get(2,0)[0] + 270));
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
        SparkFunOTOS.Pose2D position = otos.getPosition();
        xEstimate.put(0,0,position.x);
        xEstimate.put(1,0,position.y);
        xEstimate.put(2,0,position.h - 270);
        xEstimate.put(2,0,wrapAngle(xEstimate.get(2,0)[0]));

        //update Covariances
        SparkFunOTOS.Pose2D stdDev = otos.getVelocityStdDev();
        Mat stdDevM = (new Mat(3, 3, CvType.CV_64FC1, new Scalar(0)));
        stdDevM.put(0,0,Math.pow(stdDev.x, 2));
        stdDevM.put(1,1,Math.pow(stdDev.y, 2));
        stdDevM.put(2,2,Math.pow(stdDev.h, 2));
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
