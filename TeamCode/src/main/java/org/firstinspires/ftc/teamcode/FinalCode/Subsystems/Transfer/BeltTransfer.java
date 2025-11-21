package org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Transfer;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BeltTransfer {
    public CRServo transferServo;

    public BeltTransfer(HardwareMap hardwareMap) {
        transferServo = hardwareMap.get(CRServo.class, "transferServo");
    }

    public void up() {
        transferServo.setPower(1);
    }
    public void down() {
        transferServo.setPower(-1);
    }
    public void stop() {
        transferServo.setPower(0);
    }
}
