package org.firstinspires.ftc.teamcode.Experiments;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TransferTest extends OpMode {
    public CRServo transferServo;

    // comment so i can make another commit hooray
    // another comment so i can make another commit

    @Override
    public void init() {
        transferServo = hardwareMap.get(CRServo.class, "transferServo");
    }

    @Override
    public void loop() {
        transferServo.setPower(gamepad1.left_stick_y);
        telemetry.addData("Power", gamepad1.left_stick_y);
    }
}
