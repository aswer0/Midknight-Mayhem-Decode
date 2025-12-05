package org.firstinspires.ftc.teamcode.Experiments.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Outtake.Flywheel;

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

        if (currentGamepad1.cross && !previousGamepad1.cross) {
            target = 0;
        } else if (currentGamepad1.square && !previousGamepad1.square) {
            target = 3300;
        } else if (currentGamepad1.circle && !previousGamepad1.circle) {
            target = 4500;
        }

        flywheel.setTargetRPM(target);

        telemetry.addData("target RPM", target);
        telemetry.addData("is ready", flywheel.isReady());
        telemetry.update();
    }
}
