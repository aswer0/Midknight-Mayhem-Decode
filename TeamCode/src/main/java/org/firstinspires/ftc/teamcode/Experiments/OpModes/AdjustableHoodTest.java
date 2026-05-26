package org.firstinspires.ftc.teamcode.Experiments.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Sensors;

@Disabled
@TeleOp
@Config
public class AdjustableHoodTest extends OpMode {
    public static double wongShu = 0.5;
    public Servo shooter1;
    public Servo shooter2;
    @Override
    public void init() {
        shooter1 = hardwareMap.get(Servo.class, "leftHood");
        shooter2 = hardwareMap.get(Servo.class, "rightHood");
    }

    @Override
    public void loop() {
        shooter1.setPosition(wongShu);
        shooter2.setPosition(1-wongShu);
        telemetry.addData("Position", wongShu);
    }
}
