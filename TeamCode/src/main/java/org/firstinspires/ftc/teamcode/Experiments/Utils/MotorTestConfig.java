package org.firstinspires.ftc.teamcode.Experiments.Utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
@Config
public class MotorTestConfig extends OpMode {
    DcMotorEx motor;
    FtcDashboard dashboard;
    public static String config = "BL";
    public static double power = 0.5;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        motor = hardwareMap.get(DcMotorEx.class, config);
    }

    @Override
    public void loop() {
        motor.setPower(power);

        TelemetryPacket packet = new TelemetryPacket();
        dashboard.sendTelemetryPacket(packet);
    }


}
