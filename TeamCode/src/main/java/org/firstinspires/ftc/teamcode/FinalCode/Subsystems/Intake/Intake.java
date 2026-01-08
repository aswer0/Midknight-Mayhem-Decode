package org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Intake;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDFController;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Sensors;

@Config
public class Intake {

    public DcMotorEx intakeMotor;
    //public Servo intakeDoor;
    //public Sensors sensors;
    public static double INTAKE_POWER = 1;

    public double v;
    public double v_last;
    public static double slowSpeed = 0.4;

    public double motorInterval = 1000.0; // milliseconds
    public double motorSpeed = 0.5;
    public double motorRPM = 1000;

    //================= compression transfer =============

    public double kp = 1.0;
    public double ki = 1.0;
    public double kd = 1.0;
    public double kf = 1.0;

    PIDFController transfer_pid;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    public ElapsedTime timer = new ElapsedTime();

    public Intake(HardwareMap hardwareMap, Sensors sensors) {
        //this.sensors = sensors;

        intakeMotor = hardwareMap.get(DcMotorEx.class,"intakeMotor");
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        transfer_pid = new PIDFController(kp, ki, kd, kf);
//        intakeDoor = hardwareMap.get(Servo.class,"intakeDoor");
    }
    public void motorOn() {intakeMotor.setPower(INTAKE_POWER);}
    public void motorOff() {intakeMotor.setPower(0);}
    public void motorReverse() {intakeMotor.setPower(-INTAKE_POWER);}
    public void motorSlow() {intakeMotor.setPower(slowSpeed);}

    public void resetTransferTimer(){
        timer.reset();
    }
    public void continuousTransfer() {
        intakeMotor.setPower(motorSpeed);
    }
    public void intervalTransfer() {
        if (timer.milliseconds() >= motorInterval) {
            intakeMotor.setPower(motorSpeed);

            timer.reset();
        }
    }
    public void pidTransfer(boolean verbose) {
        double rpm = intakeMotor.getVelocity() * 60 / 2303.5;
        double speed = transfer_pid.calculate(motorRPM, rpm);

        if (verbose){
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Intake RPM", rpm);
        }

        intakeMotor.setPower(speed);
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
