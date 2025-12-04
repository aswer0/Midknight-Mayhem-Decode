package org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Transfer;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Intake.Intake;

@Config
public class ArmTransfer {
    public static double IDLE_POS = 0.06;
    public static double TRANSFER_POS = 0.55;
    public static int TRANSFER_WAIT_TIME1 = 400;
    public static int TRANSFER_WAIT_TIME2 = 150;
    public static int INTAKE_TIME = 400;
    public int current_shots = 0;

    public int transferStage = 5;
    private ElapsedTime timer;

    public Servo transferServo;
    Intake intake;

    public ArmTransfer(HardwareMap hardwareMap, Intake intake) {
        transferServo = hardwareMap.get(Servo.class, "transferServo");
        this.intake = intake;
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
    public boolean isReady(){
        return transferStage == 5;
    }

    public boolean update() {
        switch (transferStage) {
            case 0: //start transfer
                toTransfer();
                timer.reset();
                transferStage++;
                return false;
            case 1: //wait for arm to move up
                if (timer.milliseconds() > TRANSFER_WAIT_TIME1) {
                    timer.reset();
                    transferStage++;
                }
                return false;
            case 2: //move arm back down
                toIdle();
                timer.reset();
                transferStage++;
                return false;
            case 3: //wait for arm to reset
                if (timer.milliseconds() > TRANSFER_WAIT_TIME2) {
                    timer.reset();
                    transferStage++;
                }
                return false;
            case 4:
                if (timer.milliseconds() < INTAKE_TIME) {
                    intake.motorOn();
                } else {
                    intake.motorOff();
                    timer.reset();
                    transferStage++;
                }
                return false;
            case 5: //idle ready
                toIdle();
                current_shots++;
                return true;
        }
        return false;
    }
}
