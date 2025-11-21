package org.firstinspires.ftc.teamcode.FinalCode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDFController;

@Config
public class Park {
    HardwareMap hardwareMap;
    public DcMotorEx parkMotor;
    PIDFController hangController;
    public static double hp = 0, hi = 0, hd = 0;
    public static double tar_pos = 500;
    public static double act_pos;

    public Park(HardwareMap hardwareMap) {
        parkMotor = hardwareMap.get(DcMotorEx.class, "parkMotor");
        hangController = new PIDFController(hp, hi, hd, 0);
    }
    public void goPark() {
        act_pos = parkMotor.getCurrentPosition();
        parkMotor.setPower(-hangController.calculate(tar_pos, act_pos));
    }
}

