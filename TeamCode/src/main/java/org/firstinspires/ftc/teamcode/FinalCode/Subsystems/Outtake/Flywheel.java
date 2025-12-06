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

    public static double kp=0.01, ki=0.000, kd=0, kf=0.32;
    public static int CLOSE_RPM = 3000;
    public static int FAR_RPM = 3750;
    public static double AUTO_RPM = 3000;
    public static int THRESHOLD = 300;
    public static boolean reverseFlywheel = false;

    public DcMotorEx flywheel;
    PIDFController flywheelController = new PIDFController(kp, ki, kd, kf);

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
        List<Double> coeffs = Arrays.asList(-0.000193287, 0.0514462, -4.78688, 198.33296, -710.1902);
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
        if (targetRPM != 0) power = flywheelController.calculate(targetRPM, currentRPM);
        flywheel.setPower(power);

        if (outputDebugInfo) {
            FtcDashboard dashboard = FtcDashboard.getInstance();
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("RPM", currentRPM);
            packet.put("target RPM", targetRPM);
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
