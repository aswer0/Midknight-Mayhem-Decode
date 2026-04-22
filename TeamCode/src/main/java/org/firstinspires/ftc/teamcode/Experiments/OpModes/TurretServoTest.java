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
    public static double buffer=0.01;
    public Servo servo_UR;
    public Servo servo_UL;

    public static int servo_UR_direction = 1;
    public static int servo_UL_direction = -1;

    public static double gear_ratio = (19.0/54.0) * (19.0/40.0) * (60.0/20.0);

    public void set_ticks(double ticks){
        //Angle -> ticks
        servo_UR.setPosition(ticks * servo_UR_direction);
        servo_UL.setPosition(ticks * servo_UL_direction);
    }
    public void set_angle(double angle){
        set_ticks(gear_ratio * angle / 340);
    }

    @Override
    public void init() {
        servo_UR = hardwareMap.get(Servo.class, "servoUR");
        servo_UL = hardwareMap.get(Servo.class, "servoUL");
    }

    @Override
    public void loop() {
        servo_UR.setPosition(pos-buffer);
        servo_UL.setPosition(1-pos-buffer);

    }
}
