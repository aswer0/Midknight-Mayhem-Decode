package org.firstinspires.ftc.teamcode.Experiments.Utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Sensors;

import java.util.Locale;

@TeleOp
public class SensorTest extends OpMode {
    Sensors sensors;
    Intake intake;
    MultipleTelemetry telemetry1;
    @Override
    public void init() {
        sensors = new Sensors(hardwareMap);
        telemetry1 = new MultipleTelemetry(telemetry, (FtcDashboard.getInstance().getTelemetry()));
        intake = new Intake(hardwareMap, sensors);
        intake.doorClose();
    }

    @Override
    public void loop() {
//        telemetry1.addData("Front", sensors.getFrontColor());
        int backColor = sensors.getBackColor();
        telemetry1.addData("Back", backColor);
        telemetry1.addData("Back1", String.format(Locale.ENGLISH, "RGB %6.1f %6.1f %6.1f Distance %6.1f", sensors.back1.r, sensors.back1.g, sensors.back1.b, sensors.back1D));
        telemetry1.addData("Back2", String.format(Locale.ENGLISH, "RGB %6.1f %6.1f %6.1f Distance %6.1f", sensors.back2.r, sensors.back2.g, sensors.back2.b, sensors.back2D));

        telemetry1.addData("Mid", sensors.getMidColor());
        telemetry1.addData("Mid1", String.format(Locale.ENGLISH, "RGB %6.1f %6.1f %6.1f Distance %6.1f", sensors.mid1.r, sensors.mid1.g, sensors.mid1.b, sensors.mid1D));
        telemetry1.addData("Mid2", String.format(Locale.ENGLISH, "RGB %6.1f %6.1f %6.1f Distance %6.1f", sensors.mid2.r, sensors.mid2.g, sensors.mid2.b, sensors.mid2D));
//        if(backColor == 1) {
//            telemetry1.addData("Mid", sensors.getMidColor());
//        }
        telemetry1.addData("Current", sensors.intakeMotor.getCurrent(CurrentUnit.AMPS));
        if(gamepad1.right_bumper){
            intake.motorOn();
            intake.doorClose();
        }
        else {
            intake.motorOff();
//            intake.doorOpen();
        }
    }
}
