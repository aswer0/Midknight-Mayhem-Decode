package org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Outtake;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Hood {
    public static boolean outputDebugInfo = false;

    //MIN SERVO POSITION: 0
    //MAX_ANGLE = 50;

    public static double MAX_ANGLE = 20;
    public static double BASE_ANGLE = 30;
    public static double MAX_SERVO_POS = 0.983;
    public static Servo left_servo;
    public static Servo right_servo;

    public Hood(HardwareMap hardwareMap) {
        left_servo = hardwareMap.get(Servo.class, "hoodServoLeft");
        right_servo = hardwareMap.get(Servo.class, "hoodServoRight");
    }

    public void set_angle(double angle){
        double position = ((angle - BASE_ANGLE) / MAX_ANGLE) * MAX_SERVO_POS;

        left_servo.setPosition(position);
        right_servo.setPosition(1-position);

        if (outputDebugInfo) {
            FtcDashboard dashboard = FtcDashboard.getInstance();
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Angle", angle);
            dashboard.sendTelemetryPacket(packet);
        }

    }

    public void set_tick(double ticks){
        double position = ticks;
        left_servo.setPosition(position);
        right_servo.setPosition(1-position);
    }

}
