package org.firstinspires.ftc.teamcode.Experiments.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class ServoTest extends OpMode {
    public static double pos1 = 0.5;
    public static double pos2 = 0.5;

    Servo s1;
    Servo s2;

    @Override
    public void init() {
        s1 = hardwareMap.get(Servo.class, "s1");
        s2 = hardwareMap.get(Servo.class, "s2");
    }

    @Override
    public void loop() {
        s1.setPosition(pos1);
        s2.setPosition(pos2);
    }
}
