package org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Intake;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDFController;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Sensors;

@Config
public class Intake {

    public DcMotorEx intakeMotor;
    public DcMotorEx intakeMotorTwo;
    //public Servo intakeDoor;
    //public Sensors sensors;
    public static double INTAKE_POWER = 1;
    public static double DOOR_OPEN_POSITION = 0.9;
    public static double DOOR_CLOSE_POSITION = 1;
    public static double slowSpeed = 0.4;

    public double v;
    public double v_last;

    public static double motorInterval = 1000.0; // milliseconds
    public static double motorSpeed = 0.5;
    public static double motorRPM = 1000;

    //================= compression transfer =============

    public static double kp = 1.0;
    public static double ki = 1.0;
    public static double kd = 1.0;
    public static double kf = 1.0;
    public boolean doorOpen = false;

    PIDFController transfer_pid;
    FtcDashboard dashboard = FtcDashboard.getInstance();
//    ElapsedTime timer = new ElapsedTime();
    Servo intakeDoor;
    Sensors sensors;


    public Intake(HardwareMap hardwareMap, Sensors sensors) {
        this.sensors = sensors;

        intakeMotor = hardwareMap.get(DcMotorEx.class,"intakeMotor");
        intakeMotorTwo = hardwareMap.get(DcMotorEx.class, "intakeMotorTwo");
        intakeDoor = hardwareMap.get(Servo.class, "llServo");

        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        transfer_pid = new PIDFController(kp, ki, kd, kf);
//        intakeDoor = hardwareMap.get(Servo.class,"intakeDoor");
    }
    public void motorOn() {intakeMotor.setPower(INTAKE_POWER); intakeMotorTwo.setPower(-INTAKE_POWER);}
    public void motorOff() {intakeMotor.setPower(0); intakeMotorTwo.setPower(0);}
    public void motorReverse() {intakeMotor.setPower(-INTAKE_POWER); intakeMotorTwo.setPower(INTAKE_POWER);}
    public void motorSlow() {intakeMotor.setPower(slowSpeed); intakeMotorTwo.setPower(-slowSpeed);}
    public void doorOpen(){
        intakeDoor.setPosition(DOOR_OPEN_POSITION);
        doorOpen = true;
    }
    public void doorClose(){
        intakeDoor.setPosition(DOOR_CLOSE_POSITION);
        doorOpen = false;
    }
    public void colorSensor(){
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("left sensors", sensors.getFrontColor());
        packet.put("right sensors", sensors.getFrontColor());
        dashboard.sendTelemetryPacket(packet);
    }

//    public void resetTransferTimer(){
//        timer.reset();
//    }
    public int intakeCurrentThreshold(double threshold){
        //6.7 threshold
        if(intakeMotor.getCurrent(CurrentUnit.AMPS) > threshold) return 1;
        else return 0;
    }
    public void continuousTransfer() {
        intakeMotor.setPower(motorSpeed);
        intakeMotorTwo.setPower(-motorSpeed);
    }
    public void intervalTransfer(double timer, double onTime, double offTime) {
        timer = timer % (onTime + offTime);
        if (timer < onTime) {
            motorOn();
        } else {
            motorOff();
        }
    }
    public void pidTransfer(boolean verbose) {
        double rpm = intakeMotor.getVelocity() * 60 / 2303.5;
        double speed = transfer_pid.calculate(motorRPM, rpm);

        if (verbose){
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Intake RPM", rpm);
            dashboard.sendTelemetryPacket(packet);
        }

        intakeMotor.setPower(speed);
        intakeMotorTwo.setPower(-speed);
    }

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
