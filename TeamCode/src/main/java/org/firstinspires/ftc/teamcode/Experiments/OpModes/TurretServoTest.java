package org.firstinspires.ftc.teamcode.Experiments.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Outtake.Turret;

@TeleOp
@Config
public class TurretServoTest extends OpMode {
    Turret turret;
    public static double UR_pos=0;
    public static double UL_pos=0;
    public static double pos=0;
    public static double buffer=0.03;
    public Servo servo_UR;
    public Servo servo_UL;

    public static int servo_UR_direction = 1;
    public static int servo_UL_direction = -1;

    public static double gear_ratio = (24/12.)*(16/13.)*(30/90.);
    public static double rangeOfMotion = 265;
    public static double centerPos = 0.51;

    public void set_ticks(double ticks){
        //Angle -> ticks
        servo_UR.setPosition(ticks-buffer);
        servo_UL.setPosition(1-ticks-buffer);
    }
    public void set_angle(double angle){
        set_ticks(angle / rangeOfMotion + centerPos);
    }

    @Override
    public void init() {
        servo_UR = hardwareMap.get(Servo.class, "servoUR");
        servo_UL = hardwareMap.get(Servo.class, "servoUL");
    }

    @Override
    public void loop() {
//        servo_UR.setPosition(pos-buffer);
//        servo_UL.setPosition(1-pos-buffer);
        set_angle(pos);
    }
}
