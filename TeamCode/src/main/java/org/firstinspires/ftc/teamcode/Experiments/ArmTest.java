package org.firstinspires.ftc.teamcode.Experiments;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Transfer.ArmTransfer;

@TeleOp
@Config
public class ArmTest extends OpMode {
    public static int INTAKE_TIME = 100;

    Sensors sensors;
    Intake intake;
    ArmTransfer armTransfer;
    ElapsedTime transferTimer;
    boolean isReady = true;

    @Override
    public void init() {
        sensors = new Sensors(hardwareMap);
        intake = new Intake(hardwareMap, sensors);
        armTransfer = new ArmTransfer(hardwareMap, intake);
        transferTimer = new ElapsedTime();
    }

    @Override
    public void loop() {
//        if (gamepad1.left_bumper) {
//            armTransfer.toIdle();
//        } else if (gamepad1.right_bumper) {
//            armTransfer.toTransfer();
//        }

        isReady = armTransfer.update();
        if (gamepad1.left_bumper) {
            if (isReady) {
                armTransfer.transfer();
            }
        }
    }
}
