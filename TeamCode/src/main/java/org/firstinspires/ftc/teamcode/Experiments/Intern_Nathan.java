package org.firstinspires.ftc.teamcode.Experiments;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@TeleOp
@Config
public class Intern_Nathan extends OpMode {
    RevColorSensorV3 internNathan;
    String ball_color = "No ball";
    double e;
    NormalizedRGBA r;
    public static double purple_red = 0.012;
    public static double purple_green = 0.013;
    public static double purple_blue = 0.025;

    public static double green_red = 0;
    public static double green_green = 0.03;
    public static double green_blue = 0.02;

    public static double threshold = 0.01;

    @Override
    public void init() {
        internNathan = hardwareMap.get(RevColorSensorV3.class, "Intern Nathan");
    }

    @Override
    public void loop() {
        e = internNathan.getLightDetected();

        telemetry.addData("Light Detected:", e);
        telemetry.addData("red:", internNathan);
        telemetry.addData("green:", internNathan);
        telemetry.addData("blue:", internNathan);

        if (Math.abs(r.red - purple_red) <= threshold && Math.abs(r.red - purple_green) <= threshold  && Math.abs(r.red - purple_blue) <= threshold) {
           ball_color = "purple";
        } else if (Math.abs(r.red - green_red) <= threshold && Math.abs(r.red - green_green) <= threshold  && Math.abs(r.red - green_blue) <= threshold) {
            ball_color = "green";
        }
        else{
            ball_color = "no ball";
        }

        telemetry.addData("Ball Color:", ball_color);

    }
}
