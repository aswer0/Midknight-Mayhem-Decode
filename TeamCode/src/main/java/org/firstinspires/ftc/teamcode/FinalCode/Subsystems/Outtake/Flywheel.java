package org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Outtake;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDFController;

import java.lang.reflect.Array;
import java.util.Arrays;
import java.util.List;

@Config
public class Flywheel {
    public static boolean outputDebugInfo = false;

    public double targetRPM = 0;
//    private double targetTicks = 0;
    private double currentRPM = 0;
//    private double curentTicks = 0;

    public static double kp=0.0095, kd=0, kf=0.06; // i= 0.0002 d = 0.0006
    public double ki=0;
    //kp_new = 0.003, ki_new = 0, kd_new = 0.000112, kf_new = 0.003;
    public static double kp_tele = 0.004, ki_tele = 0, kd_tele = 0.00012, kf_tele = 0.08;
    public static double kp_auto = 0.009, ki_auto = 0, kd_auto = 0.00011, kf_auto = 0.08;
    public static int CLOSE_RPM = 2590;
    public static int FAR_RPM = 3215;
    public static int EXPERIMENTAL_RPM = 4000;
    public static double AUTO_RPM = 3000;
    public static int THRESHOLD = 150;
//    public static double TICKS_THRESHOLD = 16.7;
    public static boolean reverseFlywheel = false;
    public boolean use_gained_schedule = true;

    public DcMotorEx flywheel;
    PIDFController flywheelController = new PIDFController();

    public Flywheel(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        if (reverseFlywheel) flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void set_auto_coeffs(){
        flywheelController.set_coeffs(kp_auto, ki_auto, kd_auto, kf_auto);
    }
    public void set_tele_coeffs(){
        flywheelController.set_coeffs(kp_tele, ki_tele, kd_tele, kf_tele);
    }

    public void setTargetRPM(double target) {
        //targetTicks = RPMToTicks(target);
        targetRPM = target;
    }
    public double getCurrentRPM() {
        return (flywheel.getVelocity() / 28 * 60);
    }

    public double getTicks() {
        return flywheel.getVelocity();
    }

    public double RPMToTicks(double RPM) {
        return RPM * 28 / 60;
    }

    public boolean isReady() {
        return Math.abs(currentRPM - targetRPM) < THRESHOLD;
    }

    public void stop() {
        setTargetRPM(0);
    }
    public void shootClose() {
        setTargetRPM(CLOSE_RPM);
    }
    public void shootFar() {
        setTargetRPM(FAR_RPM);
    }
    public void shootExperiment() {
        setTargetRPM(EXPERIMENTAL_RPM);
    }
    public void shootAutoDist() {
        setTargetRPM(AUTO_RPM);
    }

    public void set_auto_rpm(double dist) {
        //List<Double> coeffs = Arrays.asList(-0.000193287, 0.0514462, -4.78688, 198.33296, -710.1902);
//        List<Double> coeffs = Arrays.asList(0.0392969, 10.62242, 1776.67);
        //List<Double> coeffs = Arrays.asList(0.0386717,-2.23064,2602.30985); //-0.314286x^{2}+58.2x+147.71429 MOST RECENT ONE
        //-0.000233344x^{4}+0.0791499x^{3}-9.85582x^{2}+546.54941x-8603.33051
        List<Double> coeffs = Arrays.asList(0.0390004, 0.510431, 2398.43221);

        int n = coeffs.size();
        double rpm = 0;

        for (int i = 0; i < n; i++) {
            rpm += coeffs.get(i) * Math.pow(dist, n - 1 - i);
        }

        AUTO_RPM = rpm;
    }

    public void update() {
        currentRPM = getCurrentRPM();
//        curentTicks = getTicks();
        double power = 0;
        if (targetRPM != 0){
            if (use_gained_schedule) {
                power = flywheelController.calculate_gain_schedule(targetRPM, currentRPM, 0);
            }
            else {
                power = flywheelController.calculate(targetRPM, currentRPM);
//                if (curentTicks < targetTicks - TICKS_THRESHOLD) {
//                    power = 1;
//                } else if (curentTicks > targetTicks + TICKS_THRESHOLD) {
//                    power = 0;
//                } else {
//                    power = kf;
//                }
            }
        }
        power = Math.max(power, 0);
        flywheel.setPower(power);

        if (outputDebugInfo) {
            FtcDashboard dashboard = FtcDashboard.getInstance();
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("RPM", currentRPM);
            packet.put("target RPM", targetRPM);
//            packet.put("curret ticks", curentTicks);
//            packet.put("target ticks", targetTicks);
            packet.put("Power", power);
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
