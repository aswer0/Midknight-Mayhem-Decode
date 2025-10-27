package org.firstinspires.ftc.teamcode.FinalCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Experiments.Sensors;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake.Flywheel;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Transfer.ArmTransfer;

@TeleOp
public class FinalTeleop extends OpMode {
    Odometry odo;
    WheelControl drive;
    Sensors sensors;
    Intake intake;
    Flywheel flywheel;
    ArmTransfer armTransfer;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();

    int turnPower = 1;
    int drivePower = 1;
    boolean useKalmanOdo = false;
    boolean isTransferReady = true;

    @Override
    public void init() {
        odo = new Odometry(hardwareMap, telemetry, 0, 0, 0);
        drive = new WheelControl(hardwareMap, odo);
        sensors = new Sensors(hardwareMap);
        intake = new Intake(hardwareMap, sensors);
        flywheel = new Flywheel(hardwareMap);
        armTransfer = new ArmTransfer(hardwareMap);
    }

    @Override
    public void loop() {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);
        odo.update();
        flywheel.update();
        isTransferReady = armTransfer.update();

        if (currentGamepad1.options && !previousGamepad1.options) {
            //set heading to 0
        }

        drive.drive(-gamepad1.left_stick_y, -1.2 * gamepad1.left_stick_x, gamepad1.right_stick_x * turnPower, -Math.toRadians(-odo.get_heading(useKalmanOdo)), drivePower);

        if (currentGamepad1.right_bumper) {
            intake.motorOn();
        } else if (currentGamepad1.right_trigger > 0.3) {
            //reverse intake
        } else {
            //stop intake
        }

        if (currentGamepad1.cross && !previousGamepad1.cross) {
            flywheel.stop();
        } else if (currentGamepad1.square && !previousGamepad1.square) {
            flywheel.shootClose();
        } else if (currentGamepad1.circle && !previousGamepad1.circle) {
            flywheel.shootFar();
        }

        if (flywheel.isReady()) {
            gamepad1.rumble(100);
        }

        if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
            if (isTransferReady) armTransfer.transfer();
        }
    }
}
