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

    private double targetRPM = 0;
    private double currentRPM = 0;

    public static double kp=0.0095, ki=0, kd=0, kf=0.06; // i= 0.0002 d = 0.0006
    public double kp_new = 0.001, ki_new = 0, kd_new = 0.075, kf_new = 0;
    public static int CLOSE_RPM = 2690;
    public static int FAR_RPM = 3350;
    public static double AUTO_RPM = 3000;
    public static int THRESHOLD = 150;
    public static boolean reverseFlywheel = false;
    public boolean use_gained_schedule = false;

    public DcMotorEx flywheel;
    PIDFController flywheelController = new PIDFController(kp, ki, kd, kf);
    PIDFController testFlywheelController = new PIDFController(kp_new, ki_new, kd_new, kf_new);

    public Flywheel(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        if (reverseFlywheel) flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setTargetRPM(double target) {
        targetRPM = target;
    }
    public double getCurrentRPM() {
        return (flywheel.getVelocity() / 28 * 60);
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
    public void shootAutoDist() {
        setTargetRPM(AUTO_RPM);
    }

    public void set_auto_rpm(double dist) {
        //List<Double> coeffs = Arrays.asList(-0.000193287, 0.0514462, -4.78688, 198.33296, -710.1902);
//        List<Double> coeffs = Arrays.asList(0.0392969, 10.62242, 1776.67);
        List<Double> coeffs = Arrays.asList(0.0386717,-2.23064,2602.30985); //-0.314286x^{2}+58.2x+147.71429
        //-0.000233344x^{4}+0.0791499x^{3}-9.85582x^{2}+546.54941x-8603.33051

        int n = coeffs.size();
        double rpm = 0;

        for (int i = 0; i < n; i++) {
            rpm += coeffs.get(i) * Math.pow(dist, n - 1 - i);
        }

        AUTO_RPM = rpm;
    }

    public void update() {
        currentRPM = getCurrentRPM();
        double power = 0;
        if (targetRPM != 0){
            if (use_gained_schedule) {
                power = testFlywheelController.calculate_gain_schedule(targetRPM, currentRPM, 0);
            }
            else {
                power = flywheelController.calculate(targetRPM, currentRPM);
            }
        }
        power = Math.max(power, 0);
        flywheel.setPower(power);

        if (outputDebugInfo) {
            FtcDashboard dashboard = FtcDashboard.getInstance();
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("RPM", currentRPM);
            packet.put("target RPM", targetRPM);
            packet.put("Power", power);
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
