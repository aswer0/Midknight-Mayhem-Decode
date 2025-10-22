package org.firstinspires.ftc.teamcode.Experiments.Drivetrain;

import static org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry.llPosition;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;

@Config
@TeleOp
public class LimelightTest extends LinearOpMode {
    Servo llServo;
    public static int tagId = 24;
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
            LLResult result = limelight.getLatestResult();
            if(result.getTimestamp() == lastTimestamp) continue;
            if(result.isValid()) continue;
            if(result == null) continue;
            lastTimestamp = result.getTimestamp();

            TelemetryPacket packet = new TelemetryPacket();

            for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                if (tag.getFiducialId() != tagId) continue;
                Pose3D position = tag.getCameraPoseTargetSpace();
                packet.put("X", position.getPosition().x/0.9144*36);
                packet.put("Y", position.getPosition().z/0.9144*36);
                packet.put("H", position.getOrientation().getPitch(AngleUnit.DEGREES));
                break;
            }
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
