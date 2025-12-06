package org.firstinspires.ftc.teamcode.Experiments.Utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDFController;

@TeleOp
@Config
public class MotorTest extends OpMode {
    DcMotorEx motor;
    FtcDashboard dashboard;
    static public double power = 1d;
    PIDFController controller;
    static public PIDFCoefficients coefficients = new PIDFCoefficients(0.01,0,0,0);

    @Override
    public void init() {
        controller = new PIDFController(coefficients);
        dashboard = FtcDashboard.getInstance();
        motor = hardwareMap.get(DcMotorEx.class, "flywheel");
        //motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        //motor.setVelocity(power, AngleUnit.DEGREES);

        //motor.setPower(controller.calculate((motor.getVelocity()/28 * 60), power));
        motor.setPower(power);
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("RPM", motor.getVelocity()/28 * 60); // REV HD hex has 28 counts per revolution
        dashboard.sendTelemetryPacket(packet);
    }


}
