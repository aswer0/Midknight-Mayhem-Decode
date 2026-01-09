package org.firstinspires.ftc.teamcode.Experiments.Utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDFController;

@TeleOp
@Config
public class MotorTestConfig extends OpMode {
    DcMotorEx motor;
    FtcDashboard dashboard;
    public static String config = "BL";
    public static double power = 0.5;

    public static double kp=0.01, ki=0.000, kd=0, kf=0.32;
    public static int RPM = 3000;

    PIDFController flywheelController = new PIDFController(kp, ki, kd, kf);
    ElapsedTime timer;
    double time_to_max_speed;
    boolean update_time = true;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        motor = hardwareMap.get(DcMotorEx.class, config);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotor.Direction.REVERSE);
        timer = new ElapsedTime();
    }
    @Override
    public void init_loop(){
        timer.reset();
    }

    @Override
    public void loop() {
        double currentRPM = motor.getVelocity() / 28 * 60;
        power = flywheelController.calculate(RPM, currentRPM);
        power = Math.max(power, 0);
        motor.setPower(power);

        if (update_time){
            time_to_max_speed = timer.milliseconds();
        }

        if (currentRPM >= RPM){
            update_time = false;
        }

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("RPM", currentRPM);
        packet.put("time", time_to_max_speed);
        dashboard.sendTelemetryPacket(packet);
    }


}
