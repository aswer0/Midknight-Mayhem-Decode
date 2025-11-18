package org.firstinspires.ftc.teamcode.Experiments.Subsystems.Drivetrain;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Camera;

public class Turret {
    PIDFCoefficients coefficients = new PIDFCoefficients();
    public enum Alliance {
        RED,
        BLUE
    }
    Limelight3A limelight;
    public Turret(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }
    public void setPosition() {

    }
}
