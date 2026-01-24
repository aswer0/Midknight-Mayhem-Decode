package org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Drivetrain;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Experiments.DrivetrainExperiments.AutoTuner.Vec3d;
import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDFController;
import org.opencv.core.Point;

//
@Config
public class WheelControl {
    public static double xp = 0.25, xi = 0, xd = 0.04, xf = 0;
    public static double yp = 0.3, yi = 0, yd = 0.04, yf = 0;
    public static double hp = 0.03, hi = 0, hd = 0.003, hF = 0;

    PIDFController x_controller;
    PIDFController y_controller;
    PIDFController h_controller;

    public DcMotorEx BR;
    public DcMotorEx BL;
    public DcMotorEx FR;
    public DcMotorEx FL;
    public double vf = 0;
    public double hf = 0;
    public double rf = 0;
    private VoltageSensor voltageSensor;

    Odometry odometry;
    public HardwareMap hardwareMap;

    DriveCorrection driveCorrection;
    double target_angle = 0;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    public WheelControl(HardwareMap hardwareMap, Odometry odometry) {
        this.BR = hardwareMap.get(DcMotorEx.class, "BR");
        this.BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.FR = hardwareMap.get(DcMotorEx.class, "FR");
        this.FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.BL = hardwareMap.get(DcMotorEx.class, "BL");
        this.BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.FL = hardwareMap.get(DcMotorEx.class, "FL");
        this.FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        this.BL.setDirection(DcMotorEx.Direction.REVERSE);
        this.FL.setDirection(DcMotorEx.Direction.REVERSE);

        this.odometry = odometry;
        this.voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        this.driveCorrection = new DriveCorrection(odometry);
        this.hardwareMap = hardwareMap;

        x_controller = new PIDFController(xp, xi, xd, xf);
        y_controller = new PIDFController(yp, yi, yd, yf);
        h_controller = new PIDFController(hp, hi, hd, hF);
    }

    public void update_gains(Vec3d p, Vec3d i, Vec3d d){
        xp = p.x;
        yp = p.y;
        hp = p.h;

        xi = i.x;
        yi = i.y;
        hi = i.h;

        xd = d.x;
        yd = d.y;
        hd = d.h;
    }
    public void original_gains(){
        xp = 0.25; xi = 0; xd = 0.01;
        yp = 0.3; yi = 0; yd = 0.01;
        hp = 0.03; hi = 0; hd = 0.003;
    }

    public void setF(double vf, double hf, double rf) {
        this.vf = vf;
        this.hf = hf;
        this.rf = rf;
    }

    public void setPowers(double BL, double BR, double FL, double FR, double power) {
        double max = 1; // max motor power
        max = Math.max(BL, max);
        max = Math.max(BR, max);
        max = Math.max(FL, max);
        max = Math.max(FR, max); // Detect the motor with the most power
        this.BL.setPower(power * (BL/max));
        this.BR.setPower(power * (BR/max));
        this.FL.setPower(power * (FL/max));
        this.FR.setPower(power * (FR/max)); // We divide all values by the maximum one so they do not reach one.
    }

    /**
     *
     * @param forward Y component of the vector (robot oriented)
     * @param right   X component of the vector (robot oriented)
     * @param rotate  Rotation velocity (radians)
     * @param heading   The angle for where to rotate the thing. Get from odometry. (field oriented)
     */
    public void drive(double forward, double right, double rotate, double heading, double power) {
        double newX = right*Math.cos(heading) - forward*Math.sin(heading);
        double newY = right*Math.sin(heading) + forward*Math.cos(heading);

        double BLPower = newY + newX + rotate;
        double BRPower = newY - newX - rotate;
        double FLPower = newY - newX + rotate;
        double FRPower = newY + newX - rotate;
        //setPowers(BLPower, BRPower, FLPower, FRPower, power);

        double max = 1;
        max = Math.max(BLPower, max);
        max = Math.max(BRPower, max);
        max = Math.max(FLPower, max);
        max = Math.max(FRPower, max); // Detect the motor with the most power
        if (!(BLPower==0)) {
            this.BL.setPower(power * (BLPower/max) + this.vf*BLPower/Math.abs(BLPower));
        } else {
            this.BL.setPower(0);
        }
        if (!(BRPower==0)) {
            this.BR.setPower(power * (BRPower/max) + this.vf*BRPower/Math.abs(BRPower));
        } else {
            this.BR.setPower(0);
        }if (!(FLPower==0)) {
            this.FL.setPower(power * (FLPower/max) + this.vf*FLPower/Math.abs(FLPower));
        } else {
            this.FL.setPower(0);
        }if (!(FRPower==0)) {
            this.FR.setPower(power * (FRPower/max) + this.vf*FRPower/Math.abs(FRPower));
        } else {
            this.FR.setPower(0);
        }
    }

    public void drive_relative(double forward, double right, double rotate_power, double max_power) {
        /*
        Positive rotate_power is CCW, negative is CW

        Wheel drive directions
        back back
        back back

        Wheel diagonals
        \ /
        / \
         */

        // Make sure forward and right are <= 1
        double power_scale = max_power/Math.max(max_power, Math.max(forward, right));
        forward *= power_scale;
        right *= power_scale;

        // Add feedforwards
        forward += Math.signum(forward)*vf;
        right += Math.signum(right)*hf;
        rotate_power += Math.signum(rotate_power)*rf;

        // Calculate motor powers
        double BLPower = -forward + right + rotate_power;
        double BRPower = -forward - right - rotate_power;
        double FLPower = -forward - right + rotate_power;
        double FRPower = -forward + right - rotate_power;

        // Get max power to make sure powers <= max_power
        double new_max_power = 1;
        new_max_power = Math.max(Math.abs(BLPower), new_max_power);
        new_max_power = Math.max(Math.abs(BRPower), new_max_power);
        new_max_power = Math.max(Math.abs(FLPower), new_max_power);
        new_max_power = Math.max(Math.abs(FRPower), new_max_power);

        // Set powers
        this.BL.setPower(BLPower/new_max_power);
        this.BR.setPower(BRPower/new_max_power);
        this.FL.setPower(FLPower/new_max_power);
        this.FR.setPower(FRPower/new_max_power);
    }

    public void stop() {
        this.BL.setPower(0);
        this.BR.setPower(0);
        this.FL.setPower(0);
        this.FR.setPower(0);
    }

    public void drive_angle(double strafe_angle, double rotate_power, double drive_power, double robot_heading) {
        /*
        Everything is in degrees
        Strafe angle is relative to field, not robot
        Power only comes from variable and not forward/right
        Useful if you want to drive at an exact power
        Rotation is added after drive
        Angles are standard (0 is positive x-axis, 90 is positive y-axis)
        Useful for driving given power and angle (polar)
        */

        // Turn strafe angle heading clockwise
        double theta = Math.toRadians(strafe_angle-robot_heading);

        // Convert angle and power to relative drive
        double forward = drive_power*Math.cos(theta);
        double right = -drive_power*Math.sin(theta);

        // Drive relatively
        drive_relative(forward, right, rotate_power, 1);
    }

    public void drive_limit_power(double drive_x, double drive_y, double rotate_power, double max_drive_power, double robot_heading) {
        /*
        Everything is in degrees
        Similar to drive but power is used as max
        Rotation is added after drive
        Angles are standard (0 is positive x-axis, 90 is positive y-axis)
        Useful for driving given x and y powers (rectangular)
         */

        // Convert x and y relative to robot forward and right
        robot_heading = Math.toRadians(robot_heading);
        double forward = drive_x*Math.cos(robot_heading) + drive_y*Math.sin(robot_heading);
        double right = drive_x*Math.sin(robot_heading) - drive_y*Math.cos(robot_heading);

        // Drive relatively
        drive_relative(forward, right, rotate_power, max_drive_power);
    }

    public void change_mode(DcMotor.ZeroPowerBehavior mode){
        this.BR.setZeroPowerBehavior(mode);
        this.FR.setZeroPowerBehavior(mode);
        this.BL.setZeroPowerBehavior(mode);
        this.FL.setZeroPowerBehavior(mode);
    }

    public void correction_drive(double left_stick_y, double left_stick_x, double rotate, double angle, double powerLevel, boolean use_kalman){
        if(rotate != 0) {
            target_angle = odometry.get_heading(use_kalman);
            this.drive(
                    left_stick_y, left_stick_x,
                    rotate,
                    angle,
                    powerLevel
            );
        }

        else {
            this.change_mode(DcMotor.ZeroPowerBehavior.BRAKE);

            this.drive(
                    left_stick_y, left_stick_x,
                    driveCorrection.stable_correction(target_angle, use_kalman),
                    angle,
                    powerLevel
            );
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Target angle", target_angle);
            (FtcDashboard.getInstance()).sendTelemetryPacket(packet);
        }
    }

    public boolean drive_to_point(Point point, double target_h, double power, double dist_thresh, boolean use_kalman){
        double rx = odometry.get_x(use_kalman);
        double ry = odometry.get_y(use_kalman);
        double rh = odometry.get_heading(use_kalman);

        double y = x_controller.calculate(point.x, rx);
        double x = y_controller.calculate(point.y, ry);
        double h = h_controller.calculate_heading(target_h, rh);

        double nominalVoltage = 13.0;
        double currentVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        double voltageComp = nominalVoltage / currentVoltage;
        double compensatedPower = power * voltageComp;
        compensatedPower = Math.min(compensatedPower, 1.0);

        this.drive(-y, -x, h, -Math.toRadians(-odometry.get_heading(use_kalman)), compensatedPower);

        double error = Math.sqrt((rx-point.x)*(rx-point.x) + (ry-point.y)*(ry-point.y));

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("error from target", error);
        (FtcDashboard.getInstance()).sendTelemetryPacket(packet);

        return error <= dist_thresh;
    }
}
