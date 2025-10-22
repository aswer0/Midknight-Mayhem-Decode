package org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Experiments.Sensors;

@Config
public class Intake {

    public DcMotorEx intakeMotor;
    public Servo intakeDoor;
    public Sensors sensors;

    public Intake(HardwareMap hardwareMap, Sensors sensors) {
        this.sensors = sensors;

        intakeMotor = hardwareMap.get(DcMotorEx.class,"intakeMotor");
        intakeDoor = hardwareMap.get(Servo.class,"intakeDoor");
    }
    public void motorOn() {intakeMotor.setPower(-1.0);}
    public void runIntake() {
        int color = sensors.getFrontColor();

        if (color == 1) {
            intakeDoor.setPosition(0.75);
        } else if (color == 2) {
            intakeDoor.setPosition(0.25);
        } else {
            intakeDoor.setPosition(0.75);
        }
    }
}
