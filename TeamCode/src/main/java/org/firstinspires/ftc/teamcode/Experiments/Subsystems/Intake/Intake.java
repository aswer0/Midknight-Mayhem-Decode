package org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Experiments.Sensors;

@Config
public class Intake {

    public DcMotorEx intakeMotor;
    //public Servo intakeDoor;
    //public Sensors sensors;

    public double v;
    public double v_last;
    public static double slowSpeed = 0.4;

    public Intake(HardwareMap hardwareMap, Sensors sensors) {
        //this.sensors = sensors;

        intakeMotor = hardwareMap.get(DcMotorEx.class,"intakeMotor");
//        intakeDoor = hardwareMap.get(Servo.class,"intakeDoor");
    }
    public void motorOn() {intakeMotor.setPower(1);}
    public void motorOff() {intakeMotor.setPower(0);}
    public void motorReverse() {intakeMotor.setPower(-1);}
    public void motorSlow() {intakeMotor.setPower(slowSpeed);}

    public void runIntake() {
        //int color = sensors.getFrontColor();

//        if (color == 1) {
//            intakeDoor.setPosition(0.75);
//        } else if (color == 2) {
//            intakeDoor.setPosition(0.25);
//        } else {
//            intakeDoor.setPosition(0.75);
//        }
    }

    public boolean jamDetect() {
        v = intakeMotor.getVelocity();

        if (v_last - v < 500) {
            v_last = v;
            return true;
        } else {
            v_last = v;
            return false;
        }
    }
}
