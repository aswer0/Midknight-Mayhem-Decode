package org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {
    private static int TICKS_PER_ROTATION = 1260; //28*15*3
    public DcMotorEx turret;

    public Turret(HardwareMap hardwareMap, boolean resetEncoder) {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (resetEncoder) turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


}
