package org.firstinspires.ftc.teamcode.Experiments.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Outtake.Flywheel;

@TeleOp
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
    }

    @Override
    public void loop() {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        flywheel.update();

        if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
            target -= 50;
        } else if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
            target += 50;
        }

        if (currentGamepad1.cross && !previousGamepad1.cross) {
            target = 0;
        } else if (currentGamepad1.square && !previousGamepad1.square) {
            target = 3300;
        } else if (currentGamepad1.circle && !previousGamepad1.circle) {
            target = 4500;
        }

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
