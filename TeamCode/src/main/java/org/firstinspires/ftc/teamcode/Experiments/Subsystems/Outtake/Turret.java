package org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDFController;

@Config
public class Turret {
    private static int TICKS_PER_ROTATION = 1260; //28*15*3
    private static int TICKS_PER_DEGREE = TICKS_PER_ROTATION/360;
    public DcMotorEx turret;

    public static double kp=0.4, ki=0, kd=0.25, kf=0;
    double target_angle;
    PIDFController controller;

    public Turret(HardwareMap hardwareMap, boolean resetEncoder) {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (resetEncoder) turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        controller = new PIDFController(kp, ki, kd, kf);

    }
    public double getAngle(){
        return turret.getCurrentPosition()/TICKS_PER_DEGREE;
    }
    public double getTicks(){
        return turret.getCurrentPosition();
    }

    public void setAngle(double target_angle){
        this.target_angle = target_angle;
    }

    public void update(){
        double power = controller.calculate(target_angle, getAngle());
        turret.setPower(power);
    }


}
