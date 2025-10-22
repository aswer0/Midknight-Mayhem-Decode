package org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp
public class NewFlywheelTest extends OpMode {
    int target = 0;

    Flywheel flywheel;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();

    @Override
    public void init() {
        flywheel = new Flywheel(hardwareMap);
    }

    @Override
    public void loop() {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        flywheel.update();

        if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
            target -= 50;
        } else if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
            target += 50;
        }
        flywheel.setTargetRPM(target);

        telemetry.addData("target RPM", target);
    }
}
