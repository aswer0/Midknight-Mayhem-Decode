package org.firstinspires.ftc.teamcode.Experiments;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Transfer.ArmTransfer;

@TeleOp
public class ArmTest extends OpMode {
    ArmTransfer armTransfer;

    @Override
    public void init() {
        armTransfer = new ArmTransfer(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.left_bumper) {
            armTransfer.toIdle();
        } else if (gamepad1.right_bumper) {
            armTransfer.toTransfer();
        }
    }
}
