package org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Experiments.Sensors;

@Config
public class LED {
    int num_balls = 0;
    //    boolean fwheel_speed = false;
    Sensors sensors;
    RevBlinkinLedDriver botLED;

    int sensorFront;
    int sensorMid;
    int sensorBack;
    int sensorFrontprev = 0;
    int sensorMidprev = 0;
    int sensorBackprev = 0;

    public LED(HardwareMap hardwareMap, Sensors sensors) {
        this.sensors = sensors;

        botLED = hardwareMap.get(RevBlinkinLedDriver.class, "LED");
    }
//
    public void ballLED() {
        sensorFront = sensors.getFrontColor();
        sensorMid = sensors.getMidColor();
        sensorBack = sensors.getBackColor();

        if (sensorFront != 0 && sensorFrontprev == 0) {
            num_balls++;
        } else if (sensorFront == 0 && sensorFrontprev != 0) {
            num_balls--;
        }

        if (sensorMid != 0 && sensorMidprev == 0) {
            num_balls++;
        } else if (sensorMid == 0 && sensorMidprev != 0) {
            num_balls--;
        }
        if (sensorBack != 0 && sensorBackprev == 0) {
            num_balls++;
        } else if (sensorBack == 0 && sensorBackprev != 0) {
            num_balls--;
        }

        num_balls = Math.max(0, Math.min(num_balls, 3));

        sensorFrontprev = sensorFront;
        sensorMidprev = sensorMid;
        sensorBackprev = sensorBack;

        // sets the LED color
        // RED == 0 ball
        // BLUE == 1 ball
        // GREEN == 2 balls
        // YELLOW == 3 balls
        if (num_balls == 0) {
            botLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        } else if (num_balls == 1) {
            botLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        } else if (num_balls == 2) {
            botLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        } else {
            botLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
        }
    }
}

