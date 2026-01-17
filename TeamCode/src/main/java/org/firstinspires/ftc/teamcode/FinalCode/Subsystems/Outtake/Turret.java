package org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Outtake;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Experiments.DrivetrainExperiments.Camera;
import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDFController;
import org.firstinspires.ftc.teamcode.FinalCode.FinalTeleop.Alliance;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Drivetrain.Odometry;

@Config
public class Turret {

    public static int TICKS_PER_ROTATION = 1572; //28*15*3
    public static double TICKS_PER_DEGREE = 3.5;
    public static double MAX_DEGREES = 100;
    public static double STOP_THRESHOLD = 0;
    public DcMotorEx turret;
//    public static PIDFCoefficients autoAimCoefficients = new PIDFCoefficients(0.01,0,0,0.25);
    public static PIDFCoefficients autoAimCoefficients = new PIDFCoefficients(0.045, .0055, 0.00025, 0.2);
    //public static PIDFCoefficients autoAimCoefficients = new PIDFCoefficients(0.035, 0.0055, 0.00045, 0.28);;
            //new PIDFCoefficients(0.045, 0.0075, 0.0003, 0.3);
            //new PIDFCoefficients(0.05, 0, 0, 0.4); // new PIDFCoefficients(0.01, 0, 0, 0.3);
    // auto aim: f = 0.43, p = 0.015
    double target_angle;
    public boolean autoAiming = false;
    PIDFController controller;
    Camera camera;
    HardwareMap hardwareMap;
    public Alliance alliance;
    public static boolean outputDebugInfo = true;
    public double CURRENT_VOLTAGE;
    ElapsedTime looptimes;

    double pastEncoder = 0;
    double estimatedTagAngle = Double.POSITIVE_INFINITY;
    double lastResultTime = Double.POSITIVE_INFINITY;
    ElapsedTime lastSeenAt = new ElapsedTime();
    PIDFCoefficients coefficients = null;
    public Odometry odometry;
    public static double[] redShootPoint = {134,136};
    public static double[] blueShootPoint = {10,136};

    public Turret(HardwareMap hardwareMap, Camera camera, Odometry odometry, Alliance alliance, boolean resetEncoder, PIDFCoefficients coefficients) {
        this.alliance = alliance;
        this.camera = camera;
        this.odometry = odometry;
        this.coefficients = coefficients;
        this.hardwareMap = hardwareMap;
        turret = hardwareMap.get(DcMotorEx.class, "turret");
//        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        if (resetEncoder) {
            turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        controller = new PIDFController(coefficients);
        looptimes = new ElapsedTime();

    }
    public Turret(HardwareMap hardwareMap, Camera camera, Odometry odometry, Alliance alliance, boolean resetEncoder) {
        this(hardwareMap,camera,odometry,alliance,resetEncoder, autoAimCoefficients);
    }
    public Turret(HardwareMap hardwareMap, Camera camera, boolean resetEncoder) {
        this(hardwareMap, camera, new Odometry(hardwareMap, new Telemetry() {
            @Override
            public Item addData(String caption, String format, Object... args) {
                return null;
            }

            @Override
            public Item addData(String caption, Object value) {
                return null;
            }

            @Override
            public <T> Item addData(String caption, Func<T> valueProducer) {
                return null;
            }

            @Override
            public <T> Item addData(String caption, String format, Func<T> valueProducer) {
                return null;
            }

            @Override
            public boolean removeItem(Item item) {
                return false;
            }

            @Override
            public void clear() {

            }

            @Override
            public void clearAll() {

            }

            @Override
            public Object addAction(Runnable action) {
                return null;
            }

            @Override
            public boolean removeAction(Object token) {
                return false;
            }

            @Override
            public void speak(String text) {

            }

            @Override
            public void speak(String text, String languageCode, String countryCode) {

            }

            @Override
            public boolean update() {
                return false;
            }

            @Override
            public Line addLine() {
                return null;
            }

            @Override
            public Line addLine(String lineCaption) {
                return null;
            }

            @Override
            public boolean removeLine(Line line) {
                return false;
            }

            @Override
            public boolean isAutoClear() {
                return false;
            }

            @Override
            public void setAutoClear(boolean autoClear) {

            }

            @Override
            public int getMsTransmissionInterval() {
                return 0;
            }

            @Override
            public void setMsTransmissionInterval(int msTransmissionInterval) {

            }

            @Override
            public String getItemSeparator() {
                return "";
            }

            @Override
            public void setItemSeparator(String itemSeparator) {

            }

            @Override
            public String getCaptionValueSeparator() {
                return "";
            }

            @Override
            public void setCaptionValueSeparator(String captionValueSeparator) {

            }

            @Override
            public void setDisplayFormat(DisplayFormat displayFormat) {

            }

            @Override
            public Log log() {
                return null;
            }
        }, 72,72,0), Alliance.red, resetEncoder);
    }
    public double getAngle(){
        return (double)turret.getCurrentPosition()/TICKS_PER_DEGREE;
    }
    public double getTicks(){
        return turret.getCurrentPosition();
    }

    public void setAngle(double target_angle){
        this.target_angle = target_angle;
    }
    public double v_compensate(double power){
        double nominalVoltage = 12;
        double voltageComp = nominalVoltage / CURRENT_VOLTAGE;
        double compensatedPower = power * voltageComp;
        compensatedPower = Math.min(compensatedPower, 1.0);
        return Math.max(compensatedPower, -1.0);
    }

    public double update(){
        double power = 0;
//        boolean shouldAutoAim = true;
//        double angle = Double.POSITIVE_INFINITY;
//        LLResult result = camera.limelight.getLatestResult();
//        boolean interpolating = false;
//        if(result != null && lastResultTime != result.getTimestamp()) {
//            for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
//                if (alliance == Alliance.red && tag.getFiducialId() == 24) {
//                    angle = tag.getTargetXDegrees(); //Math.toDegrees(Math.atan(-pose.getPosition().x/pose.getPosition().z));
//                    estimatedTagAngle = angle;
//                    pastEncoder = turret.getCurrentPosition();
//                    lastSeenAt = new ElapsedTime();
//                    break;
//                } else if (tag.getFiducialId() == 20 && alliance == Alliance.blue) {// blue
//                    angle = tag.getTargetXDegrees();
//                    estimatedTagAngle = angle;
//                    pastEncoder = turret.getCurrentPosition();
//                    lastSeenAt = new ElapsedTime();
//                    break;
//                }
//            }
//            lastResultTime = result.getTimestamp();
//
//        }
//        if (angle == Double.POSITIVE_INFINITY) { // no tag detected
//            interpolating = true;
//            angle = (estimatedTagAngle + (pastEncoder - turret.getCurrentPosition())/TICKS_PER_DEGREE);
//        }
//        if(angle == Double.POSITIVE_INFINITY) {
//            shouldAutoAim = false;
//        }
//        if(lastSeenAt.seconds() > 2) { // lose the angle when havent seen tag for a while.
//            estimatedTagAngle = Double.POSITIVE_INFINITY;
//        }

        if(autoAiming) {
//            odometry.update();

            //===================== FUTURE VEL PREDICTION =====================
            double future_x = odometry.get_x_predicted(false, true);
            double future_y = odometry.get_y_predicted(false, true);

            //==========================================

            double actual = getAngle(); //- odometry.get_heading(false);
            double target = 0;
            if(alliance == Alliance.blue) target = -Math.toDegrees(Math.atan2((blueShootPoint[1]-future_y),(blueShootPoint[0]-future_x))) +  odometry.get_heading(false);
            else target =                           -Math.toDegrees(Math.atan2((redShootPoint[1]-future_y),(redShootPoint[0]-future_x))) +  odometry.get_heading(false);
            power = controller.calculate_heading(
                    target,
                    actual, controller.gains.f * Math.signum(PIDFController.wrapError(target, getAngle())));
            power = v_compensate(power);
            if (power <= STOP_THRESHOLD && power >= -STOP_THRESHOLD){
                power = 0;
            }
            if (outputDebugInfo) {
                TelemetryPacket packet = new TelemetryPacket();
                packet.put("Auto Target", target);
                packet.put("Auto Actual", actual);
                packet.put("Auto Error", PIDFController.wrapError(target, actual));
                packet.put("Turret Power", power);
                packet.put("Turret F", controller.gains.f * Math.signum(target - getAngle()));
                packet.put("odometry", new double[]{odometry.get_x(false), odometry.get_y(false), odometry.get_heading(false)});
                //packet.put("LLResult", result.getTimestamp() - lastResultTime);
                (FtcDashboard.getInstance()).sendTelemetryPacket(packet);
//                turret.setPower(0);
//                return 0;
            }
        } else {
            power = controller.calculate_heading(target_angle, getAngle(), controller.gains.f * Math.signum(target_angle - getAngle()));
            power = Math.min(power, 1);
            power = Math.max(power, -1);

            power = v_compensate(power);
            if (power <= STOP_THRESHOLD && power >= -STOP_THRESHOLD){
                power = 0;
            }

            //turret.setPower(power);
            if(outputDebugInfo) {
                TelemetryPacket packet = new TelemetryPacket();
                packet.put("Single Actual Angle", getAngle());
                packet.put("Single Target Angle", target_angle);
                packet.put("Turret Power", power);
                (FtcDashboard.getInstance()).sendTelemetryPacket(packet);
            }
        }
        if (getAngle() < MAX_DEGREES && getAngle() > -MAX_DEGREES) {
            turret.setPower(power);
        } else {
            power = controller.calculate_heading(0, getAngle(), controller.gains.f * Math.signum(target_angle - getAngle()));
            power = Math.min(power, 1);
            power = Math.max(power, -1);
            power = v_compensate(power);
            turret.setPower(power);
        }
        return power;
    }


}
