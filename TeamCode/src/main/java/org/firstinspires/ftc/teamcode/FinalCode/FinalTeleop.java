package org.firstinspires.ftc.teamcode.FinalCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Drivetrain.WheelControl;

@TeleOp
public class FinalTeleop extends OpMode {
    Odometry odo;
    WheelControl drive;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();

    int turnPower = 1;
    int drivePower = 1;
    boolean useKalmanOdo = false;

    @Override
    public void init() {
        odo = new Odometry(hardwareMap, telemetry, 0, 0, 0);
        drive = new WheelControl(hardwareMap, odo);
    }

    @Override
    public void loop() {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);
        odo.update();

        drive.drive(gamepad1.left_stick_y, 1.2 * gamepad1.left_stick_x, -gamepad1.right_stick_x * turnPower, Math.toRadians(odo.get_heading(useKalmanOdo)), drivePower);
    }
}
