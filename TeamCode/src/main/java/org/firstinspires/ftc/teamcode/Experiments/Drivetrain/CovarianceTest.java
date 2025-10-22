package org.firstinspires.ftc.teamcode.Experiments.Drivetrain;

import static org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry.llPosition;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;

import java.util.Arrays;

@Config
@TeleOp
public class CovarianceTest extends LinearOpMode {
    Servo llServo;
    public static int tagId = 24;
    public static int numIterations = 30;
    public static boolean startCapture = false;
    public boolean prevStartCapture = true;
    public double lastTimestamp = Double.POSITIVE_INFINITY;
    Limelight3A limelight;
    FtcDashboard dashboard;

    @Override
    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        llServo = hardwareMap.get(Servo.class, "llServo");
        llServo.setPosition(llPosition);

        waitForStart();
        while(opModeIsActive()) {
            while(startCapture != prevStartCapture) {
                if(isStopRequested()) return;
            }
            TelemetryPacket packet = new TelemetryPacket();
            prevStartCapture = !startCapture;
            int iterations = 0;
            Mat samples = new Mat(3, numIterations, CvType.CV_64FC1, new Scalar(0));
            while (iterations < numIterations) {
                if(isStopRequested()) return;
                LLResult result = limelight.getLatestResult();
                if(result.getTimestamp() == lastTimestamp) continue;
                if (result == null) continue;
                if (!result.isValid()) continue;
                lastTimestamp = result.getTimestamp();
                double[] area = new double[numIterations];
                for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                    if (tag.getFiducialId() != tagId) continue;
                    Pose3D position = tag.getCameraPoseTargetSpace();

                    //TelemetryPacket packet = new TelemetryPacket();
//                packet.put("X", position.getPosition().x);
//                packet.put("Y", position.getPosition().z);
//                packet.put("H", position.getOrientation().getPitch(AngleUnit.DEGREES));

                    //packet.fieldOverlay().setFill("#FF00FF").fillText("*", position.getPosition().x, position.getPosition().z, "30px Arial", position.getOrientation().getYaw());

                    //(FtcDashboard.getInstance()).sendTelemetryPacket(packet);

//                xMeasurements[iterations] = position.getPosition().x;
//                yMeasurements[iterations] = position.getPosition().z;
//                hMeasurements[iterations] = position.getOrientation().getPitch(AngleUnit.DEGREES);
                    area[iterations] = tag.getTargetArea();
                    samples.put(0, iterations, position.getPosition().x/0.9144*36);
                    samples.put(1, iterations, position.getPosition().z/0.9144*36);
                    samples.put(2, iterations, position.getOrientation().getPitch(AngleUnit.DEGREES));
                    if(iterations % 1 == 0) packet.fieldOverlay().setFill("#000000").fillCircle(position.getPosition().x/0.9144*36,position.getPosition().z/0.9144*36 + 72,1);
                    iterations++;
                    break;
                }
            }
            Mat mean = new Mat(3, 1, CvType.CV_64FC1, new Scalar(0));
            Mat covarianceMatrix = new Mat(3, 3, CvType.CV_64FC1, new Scalar(0)); // covariance = E[(X - E[X])*(X - E[X])^T]
            Core.calcCovarMatrix(samples, covarianceMatrix, mean, Core.COVAR_NORMAL | Core.COVAR_COLS);

            // multiply by
            // [ cos(h), sin(h), 0;
            //  -sin(h), cos(h), 0;
            //        0,      0, 1;]
//            double hMean = mean.get(2, 0)[0];
//            Mat a = new Mat(3, 3, CvType.CV_64FC1, new Scalar(0));
//            a.put(0, 0, Math.cos(Math.toRadians(hMean)));
//            a.put(0, 1, Math.sin(Math.toRadians(hMean)));
//            a.put(1, 0, -Math.sin(Math.toRadians(hMean)));
//            a.put(1, 1, Math.cos(Math.toRadians(hMean)));
//            a.put(2, 2, 1);
//            Mat aT = new Mat(3, 3, CvType.CV_64FC1, new Scalar(0));
//            Core.transpose(a, aT);
//            covarianceMatrix = a.matMul(covarianceMatrix).matMul(aT);
            // Ellipse Rotation ~ -2.06628*atan2(-Y,X) + 186.08724 (prob 2*atan2(X, -Y))
            // 0.3 (uncertain) - variance on the axis perpendicular to the tag
            //
            packet.put("X", mean.get(0, 0)[0]);
            packet.put("Y", mean.get(1, 0)[0]);
            packet.put("Direction from April Tag", Math.toDegrees(Math.atan2(-mean.get(1,0)[0],mean.get(0,0)[0])));
            packet.put("H", mean.get(2, 0)[0]);


            // draw the error ellipse
            Mat eigenvalues = new Mat(); //square root of eigenvalues indicate length of axes
            Mat eigenvectors = new Mat(); // vectors indicate major/minor axes
            Core.eigen(covarianceMatrix.submat(0,2,0,2), eigenvalues, eigenvectors);

            double a = Math.sqrt(eigenvalues.get(0,0)[0]*5.991);// /0.9144*36; // first axis (first eigenvector)
            double b = Math.sqrt(eigenvalues.get(1,0)[0]*5.991);// /0.9144*36; // second axis (second eigenvector)
            packet.fieldOverlay()
                    .setFill("#FF00FF")
                    .setAlpha(0.5)
                    .setRotation(Math.atan2(eigenvectors.get(0,1)[0],eigenvectors.get(0,0)[0]))
                    .setScale(a,b)
                    .setTranslation(mean.get(0, 0)[0],mean.get(1, 0)[0] + 72)
                    .fillCircle(0,0,1);

            packet.put("Ellipse rotation", Math.toDegrees(Math.atan2(eigenvectors.get(0,1)[0],eigenvectors.get(0,0)[0])));
            packet.put("a",eigenvalues.get(0,0)[0]);
            packet.put("b",eigenvalues.get(1,0)[0]);
            //packet.put("Covariance matrix", covarianceMatrix.dump().substring(1, covarianceMatrix.dump().length() - 1).replace("\n",""));
            dashboard.sendTelemetryPacket(packet);
            telemetry.addData("c", covarianceMatrix.dump());
        }
    }
}
