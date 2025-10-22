package org.firstinspires.ftc.teamcode.Experiments.Subsystems.Transfer;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class ArmTransfer {
    public static double IDLE_POS = 0.5;
    public static double TRANSFER_POS = 0.5;
    public static int TRANSFER_WAIT_TIME1 = 500;
    public static int TRANSFER_WAIT_TIME2 = 500;

    private int transferStage = 3;
    private ElapsedTime timer;

    public Servo transferServo;

    public ArmTransfer(HardwareMap hardwareMap) {
        transferServo = hardwareMap.get(Servo.class, "transferServo");
        timer = new ElapsedTime();
    }

    public void toIdle() {
        transferServo.setPosition(IDLE_POS);
    }
    public void toTransfer() {
        transferServo.setPosition(TRANSFER_POS);
    }

    public void transfer() {
        transferStage = 0;
    }


    public boolean update() {
        switch (transferStage) {
            case 0: //start transfer
                toTransfer();
                timer.reset();
                transferStage++;
                return false;
            case 1: //wait for arm to move up
                if (timer.milliseconds() > TRANSFER_WAIT_TIME1) transferStage++;
                return false;
            case 2: //move arm back down
                toIdle();
                timer.reset();
                transferStage++;
                return false;
            case 3: //wait for arm to reset
                if (timer.milliseconds() > TRANSFER_WAIT_TIME2) transferStage++;
                return false;
            case 5: //idle ready
                return true;
        }
        return false;
    }
}
