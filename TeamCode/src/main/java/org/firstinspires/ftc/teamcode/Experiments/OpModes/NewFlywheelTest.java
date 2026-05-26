package org.firstinspires.ftc.teamcode.Experiments.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Outtake.Flywheel;

@TeleOp(group = "Test")
@Config
public class NewFlywheelTest extends OpMode {
    public static int target = 0;

    Flywheel flywheel;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void init() {
        flywheel = new Flywheel(hardwareMap);
        flywheel.use_gained_schedule = true;
        flywheel.set_tele_coeffs();
    }

    @Override
    public void loop() {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        flywheel.update();
        flywheel.setTargetRPM(target);

        telemetry.addData("target RPM", target);
        telemetry.addData("is ready", flywheel.isReady());
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Target RPM", target);
        packet.put("Actual RPM", flywheel.getCurrentRPM());
        dashboard.sendTelemetryPacket(packet);

    }
}
