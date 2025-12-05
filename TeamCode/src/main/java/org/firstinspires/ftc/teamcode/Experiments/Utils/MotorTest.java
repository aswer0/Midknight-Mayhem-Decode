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
    static public double power = 0;
    static public boolean STOP = false;
    static public String config = "FR";
    PIDFController controller;
    static public PIDFCoefficients coefficients = new PIDFCoefficients(0.01,0,0,0);

    @Override
    public void init() {
        controller = new PIDFController(coefficients);
        dashboard = FtcDashboard.getInstance();
        motor = hardwareMap.get(DcMotorEx.class, config);
        //motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        motor.setPower(power);
        if (STOP){
            power = 0;
            STOP = false;
        }
    }


}
