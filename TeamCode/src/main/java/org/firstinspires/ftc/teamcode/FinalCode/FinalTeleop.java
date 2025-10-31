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
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Transfer.BeltTransfer;
import org.opencv.core.Point;

@TeleOp
public class FinalTeleop extends OpMode {
    Odometry odo;
    WheelControl drive;
    Sensors sensors;
    Intake intake;
    Flywheel flywheel;
    BeltTransfer beltTransfer;
    //ArmTransfer armTransfer;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();

    public static Point shoot_point = new Point(60, 81);
    public static double target_shoot_heading = 135;

    double turnPower = 1;
    int drivePower = 1;
    boolean useKalmanOdo = false;
    boolean isTransferReady = true;
    boolean useDriveCorrecton = true;
    boolean pidToPoint = false;

    @Override
    public void init() {
        odo = new Odometry(hardwareMap, telemetry, 0, 0, 0);
        drive = new WheelControl(hardwareMap, odo);
        sensors = new Sensors(hardwareMap);
        intake = new Intake(hardwareMap, sensors);
        flywheel = new Flywheel(hardwareMap);
        beltTransfer = new BeltTransfer(hardwareMap);
        //armTransfer = new ArmTransfer(hardwareMap);
    }

    @Override
    public void loop() {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);
        odo.update();
        flywheel.update();
        //isTransferReady = armTransfer.update();

        if (currentGamepad1.options && !previousGamepad1.options) {
            //set heading to 0
            odo.set_heading(0);
        }
        if (currentGamepad1.share && !previousGamepad1.share) {
            //set heading to 0
            useDriveCorrecton = !useDriveCorrecton;
        }
        if (currentGamepad1.right_stick_button && !previousGamepad1.right_stick_button) {
            pidToPoint = true;
        }

        //drive
        if (useDriveCorrecton && !pidToPoint){
            drive.correction_drive(gamepad1.left_stick_y, 1.2 * gamepad1.left_stick_x, -gamepad1.right_stick_x * turnPower, -Math.toRadians(-odo.get_heading(useKalmanOdo)), drivePower, false);
        }
        else{
            if (!pidToPoint){
                drive.drive(gamepad1.left_stick_y, 1.2 * gamepad1.left_stick_x, -gamepad1.right_stick_x * turnPower, -Math.toRadians(-odo.get_heading(useKalmanOdo)), drivePower);
            }
        }
        if (pidToPoint){
            drive.drive_to_point(shoot_point, target_shoot_heading, 1, 0.5, false);
        }

        if (currentGamepad1.left_stick_button){
            drivePower = drivePower/(drivePower+1);
        }
        else{
            drivePower = 1;
        }

        //intake
        if (currentGamepad1.right_bumper) {
            intake.motorOn();
        } else if (currentGamepad1.right_trigger > 0.3) {
            intake.motorReverse();
        } else {
            //transfer
            if (currentGamepad1.left_bumper) {
//              if (isTransferReady) armTransfer.transfer();
                beltTransfer.up();
                intake.motorOn();
            } else {
                beltTransfer.stop();
                intake.motorOff();
            }
        }

        //flywheel
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
    }
}
