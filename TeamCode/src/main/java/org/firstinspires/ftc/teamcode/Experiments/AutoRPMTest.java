package org.firstinspires.ftc.teamcode.Experiments;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Drivetrain.WheelControl;
import org.opencv.core.Point;

@TeleOp
@Config
public class AutoRPMTest extends OpMode {
    WheelControl drive;
    Odometry odometry;

    public static double turnPower = 1;
    public static int drivePower = 1;

    public static boolean next_state = false;

    enum State{
        driving,
    }

    State state = State.driving;

    @Override
    public void init() {
        odometry = new Odometry(hardwareMap, telemetry, 8, 8, 0);
        drive = new WheelControl(hardwareMap, odometry);
    }

    @Override
    public void loop() {
        odometry.update();


        switch (state){
            case driving:
                drive.correction_drive(
                        gamepad1.left_stick_y,
                        1.2 * gamepad1.left_stick_x,
                        -gamepad1.right_stick_x * turnPower,
                        -Math.toRadians(-odometry.get_heading(false)),
                        drivePower, false
                );

        }
    }
}
