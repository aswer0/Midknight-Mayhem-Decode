package org.firstinspires.ftc.teamcode.FinalCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experiments.DrivetrainExperiments.Camera;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Drivetrain.GVF.VectorField;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Outtake.Flywheel;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Outtake.Turret;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Transfer.ArmTransfer;
import org.opencv.core.Point;

@Autonomous
@Config
public class FarAutoBlue extends OpMode {
    public static Point start_point = new Point(56, 8);
    public static Point park_point = new Point(30, 10);

    enum State {
        wait,
        shootBall,
        park
    }

    State state = State.wait;

    WheelControl wheelControl;
    Odometry odometry;
    VectorField vf;
    ElapsedTime timer;

    Intake intake;
    Flywheel flywheel;
    Sensors sensors;
    ArmTransfer armTransfer;
    Turret turret;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();

    public static boolean uk = false;
    public static double pidf_threshold = 0.5;
    public static double power = 0.8;
    public static double turret_angle = 67.7;
    public static double bot_angle = 180;

    public static double shoot_wait_time = 6000;

    int wait_time = 0;
    int shotCounter = 0;
    boolean prevFlywheelReady = false;

    @Override
    public void init() {
        odometry = new Odometry(hardwareMap, telemetry, start_point.x, start_point.y, bot_angle);
        wheelControl = new WheelControl(hardwareMap, odometry);

        vf = new VectorField(wheelControl, odometry, uk);
        timer = new ElapsedTime();

        sensors = new Sensors(hardwareMap);
        intake = new Intake(hardwareMap, sensors);
        flywheel = new Flywheel(hardwareMap);
        armTransfer = new ArmTransfer(hardwareMap, intake);
        turret = new Turret(hardwareMap, new Camera(hardwareMap), odometry, FinalTeleop.Alliance.blue, true);
        FinalTeleop.alliance = FinalTeleop.Alliance.blue;
    }

    @Override
    public void init_loop() {
        timer.reset();

        if (!previousGamepad1.dpad_left && currentGamepad1.dpad_left){
            wait_time--;
            wait_time = Math.max(wait_time, 0);
        }
        if (!previousGamepad1.dpad_right && currentGamepad1.dpad_right){
            wait_time++;
        }

        telemetry.addData("wait time (dpad)", wait_time);
        telemetry.update();
    }

    @Override
    public void start() {
        turret.setAngle(turret_angle);
    }

    @Override
    public void loop() {
        odometry.update();
        turret.update();
        boolean isTransferReady = armTransfer.update();

        switch (state) {
            case wait:
                if (timer.milliseconds() >= wait_time) {
                    timer.reset();
                    state = State.shootBall;
                }
                break;

            case shootBall:
                flywheel.shootFar();
                flywheel.update();

                if (timer.milliseconds() >= 1500) {
                    if (isTransferReady) armTransfer.transfer();
                }

                if (timer.milliseconds() >= shoot_wait_time) {
                    state = State.park;
                }

                break;

            case park:
                armTransfer.toIdle();
                flywheel.stop();
                flywheel.update();
                intake.motorOff();

                wheelControl.drive_to_point(park_point, bot_angle, 1, pidf_threshold, false);

                FinalTeleop.startX = park_point.x;
                FinalTeleop.startY = park_point.y;
                FinalTeleop.startHeading = bot_angle;
                break;
        }

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("turret angle (actual)", turret.getAngle());
    }
}
