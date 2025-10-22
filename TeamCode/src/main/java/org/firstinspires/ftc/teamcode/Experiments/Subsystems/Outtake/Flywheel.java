package org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDFController;

@Config
public class Flywheel {
    public static boolean outputDebugInfo = false;

    private int targetRPM = 0;
    private int currentRPM = 0;

    public static double kp=1, ki=0, kd=0;
    public static int CLOSE_RPM = 3000;
    public static int FAR_RPM = 4000;
    public static int THRESHOLD = 100;

    public DcMotorEx flywheel;
    PIDFController flywheelController = new PIDFController(kp, ki, kd, 0);

    public Flywheel(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setTargetRPM(int target) {
        targetRPM = target;
    }
    public int getCurrentRPM() {
        return (int) (flywheel.getVelocity() / 28 * 60);
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
