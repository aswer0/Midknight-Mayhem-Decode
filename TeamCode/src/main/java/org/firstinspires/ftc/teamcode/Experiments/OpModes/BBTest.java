package org.firstinspires.ftc.teamcode.Experiments.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;


@TeleOp
public class BBTest extends OpMode {
    DigitalChannel bb1;
    DigitalChannel bb2;
    @Override
    public void init() {
        bb1 = hardwareMap.get(DigitalChannel.class, "BB1");
        bb2 = hardwareMap.get(DigitalChannel.class, "BB2");
        bb1.setMode(DigitalChannel.Mode.INPUT);
        bb2.setMode(DigitalChannel.Mode.INPUT);
        bb1.setState(false);
        bb2.setState(false);
    }

    @Override
    public void loop() {
        telemetry.addData("b1",bb1.getState());
        telemetry.addData("b2",bb2.getState());
    }
}
