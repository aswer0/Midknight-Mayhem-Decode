package org.firstinspires.ftc.teamcode.Experiments;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Config
public class Intern_Nathan extends OpMode {
    RevColorSensorV3 internNathan;
    RevColorSensorV3 ceoAndrew;
    String ball_color = "No ball";
    double nathan_red_value;
    double nathan_green_value;
    double nathan_blue_value;
    double andrew_red_value;
    double andrew_green_value;
    double andrew_blue_value;

    @Override
    public void init() {
        internNathan = hardwareMap.get(RevColorSensorV3.class, "Intern Nathan");
        ceoAndrew = hardwareMap.get(RevColorSensorV3.class, "CEO Andrew");
    }

    @Override
    public void loop() {

        nathan_red_value = internNathan.red();
        nathan_green_value = internNathan.green();
        nathan_blue_value = internNathan.blue();

        andrew_red_value = ceoAndrew.red();
        andrew_green_value = ceoAndrew.green();
        andrew_blue_value = ceoAndrew.blue();

        telemetry.addData("nathan red:", nathan_red_value);
        telemetry.addData("nathan green:", nathan_green_value);
        telemetry.addData("nathan blue:", nathan_blue_value);

        telemetry.addData("andrew red:", andrew_red_value);
        telemetry.addData("andrew green:", andrew_green_value);
        telemetry.addData("andrew blue:", andrew_blue_value);

        if (nathan_red_value + nathan_green_value + nathan_blue_value >= 800) {
            if (nathan_blue_value > nathan_red_value && nathan_blue_value > nathan_green_value) {
                ball_color = "purple";
            } else if (nathan_green_value > nathan_red_value && nathan_green_value > nathan_blue_value) {
                ball_color = "green";
            }
        }
        else {
            if (andrew_red_value + andrew_green_value + andrew_blue_value >= 900) {
                if (andrew_blue_value > andrew_red_value && andrew_blue_value > andrew_green_value) {
                    ball_color = "purple";
                } else if (andrew_green_value > andrew_red_value && andrew_green_value > andrew_blue_value) {
                    ball_color = "green";
                }
            }
            else {
                ball_color = "no color";
            }
        }
        telemetry.addData("Ball Color:", ball_color);
    }
}
