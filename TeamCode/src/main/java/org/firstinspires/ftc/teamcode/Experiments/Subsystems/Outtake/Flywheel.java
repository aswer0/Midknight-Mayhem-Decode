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
    public static double kp=1, ki=0, kd=0;

    public DcMotorEx flywheel;
    PIDFController flywheelController = new PIDFController(kp, ki, kd, 0);

    public Flywheel(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setTargetRPM(int target) {
        targetRPM = target;
    }

    public void update() {
        double currentRPM = flywheel.getVelocity() / 28 * 60;
        double power = flywheelController.calculate(targetRPM, currentRPM);
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
