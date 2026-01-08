package org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Transfer;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class CompressionTransfer {
    HardwareMap hardwareMap;
    public DcMotorEx transferMotor;
    public double motorInterval = 1000.0; // milliseconds
    public double motorSpeed = 0.5;

    // for intervalTransfer
    public ElapsedTime timer = new ElapsedTime();
//    public boolean motorOn = false;

    public CompressionTransfer(HardwareMap hardwareMap) {
        transferMotor = hardwareMap.get(DcMotorEx.class, "ctMotor");
    }

    public void continuousTransfer() {
        transferMotor.setPower(motorSpeed);
    }

    public void intervalTransfer() {
        if (timer.milliseconds() >= motorInterval) {
//            motorOn = !motorOn;

            transferMotor.setPower(motorSpeed);

            timer.reset();
        }
    }
}
