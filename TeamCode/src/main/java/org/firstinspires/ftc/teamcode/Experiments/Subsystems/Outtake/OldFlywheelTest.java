package org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp
@Config
public class OldFlywheelTest extends OpMode {
    DcMotorEx flywheel;

    @Override
    public void init() {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        flywheel.setPower(gamepad1.left_stick_y);
        telemetry.addData("power", gamepad1.left_stick_y);
        telemetry.addData("Velocity", (flywheel.getVelocity()) / 28 * 60);

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("velocity",(flywheel.getVelocity()) / 28 * 60);
        (FtcDashboard.getInstance()).sendTelemetryPacket(packet);
    }
}
