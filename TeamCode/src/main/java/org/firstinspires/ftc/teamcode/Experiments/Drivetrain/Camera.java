package org.firstinspires.ftc.teamcode.Experiments.Drivetrain;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class Camera {
    public enum Pattern {
        PPG,
        PGP,
        GPP
    }
    public enum Alliance {
        RED,
        BLUE
    }
    public Limelight3A limelight;
    public Pattern detectedPattern = null;
    public Camera(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }
    public void update() {
        LLResult result = limelight.getLatestResult();
        for(LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            switch(tag.getFiducialId()) {
                case 21:
                    detectedPattern = Pattern.GPP;
                    break;
                case 22:
                    detectedPattern = Pattern.PGP;
                    break;
                case 23:
                    detectedPattern = Pattern.PPG;
                    break;
            }
        }
    }

}
