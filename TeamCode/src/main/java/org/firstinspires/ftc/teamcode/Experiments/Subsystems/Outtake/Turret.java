package org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Camera;
import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDFController;

@Config
public class Turret {

    public enum Alliance {
        RED,
        BLUE
    }
    private static int TICKS_PER_ROTATION = 1260; //28*15*3
    private static int TICKS_PER_DEGREE = TICKS_PER_ROTATION/360;
    public DcMotorEx turret;

    public static double kp=0.4, ki=0, kd=0.25, kf=0;
    public static PIDFCoefficients manualAimCoefficients = new PIDFCoefficients(kp, ki, kd, kf);
    double target_angle;
    public static boolean autoAiming = true;
    PIDFController controller;
    Camera camera;
    Alliance alliance;
    public static boolean outputDebugInfo = false;
    public Turret(HardwareMap hardwareMap, Camera camera, Alliance alliance, boolean resetEncoder) {
        this.alliance = alliance;
        this.camera = camera;
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        if (resetEncoder) turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        controller = new PIDFController(manualAimCoefficients);

    }
    public Turret(HardwareMap hardwareMap, Camera camera, boolean resetEncoder) {
        this(hardwareMap, camera, Alliance.RED, resetEncoder);
    }
    public double getAngle(){
        return (double)turret.getCurrentPosition()/TICKS_PER_DEGREE;
    }
    public double getTicks(){
        return turret.getCurrentPosition();
    }

    public void setAngle(double target_angle){
        this.target_angle = target_angle;
    }

    public double update(){
        double power;
        if(autoAiming) {
            double angle = Double.POSITIVE_INFINITY;
            LLResult result = camera.limelight.getLatestResult();
            if(result == null) { // no tag detected
                turret.setPower(0);
                return 0;
            }
            for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                if(alliance == Alliance.RED && tag.getFiducialId() == 24) {
                    Pose3D pose = tag.getCameraPoseTargetSpace();
                    angle = Math.toDegrees(Math.atan(-pose.getPosition().x/pose.getPosition().z));
                    break;
                } else if (tag.getFiducialId() == 20) {// blue
                    Pose3D pose = tag.getCameraPoseTargetSpace();
                    angle = Math.toDegrees(Math.atan(-pose.getPosition().x/pose.getPosition().z));
                    break;
                }
            }
            if(angle == Double.POSITIVE_INFINITY) { // no tag detected
                turret.setPower(0);
                return 0;
            }
            if(outputDebugInfo) {
                TelemetryPacket packet = new TelemetryPacket();
                packet.put("Tag Angle", angle);
                (FtcDashboard.getInstance()).sendTelemetryPacket(packet);
                turret.setPower(0);
                return 0;
            }
            power = controller.calculate(0, -angle);
            turret.setPower(power);
        } else {
            power = controller.calculate(target_angle, getAngle());
            turret.setPower(power);
            if(outputDebugInfo) {
                TelemetryPacket packet = new TelemetryPacket();
                packet.put("Actual Angle", getAngle());
                packet.put("Target Angle", target_angle);
                (FtcDashboard.getInstance()).sendTelemetryPacket(packet);
            }
        }
        return power;
    }


}
