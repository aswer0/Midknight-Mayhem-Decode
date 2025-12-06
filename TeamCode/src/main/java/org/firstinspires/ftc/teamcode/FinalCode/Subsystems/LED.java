package org.firstinspires.ftc.teamcode.FinalCode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Outtake.Flywheel;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Sensors;

@Config
public class LED {
    int num_balls = 0;
    int ball_color = 0;
    Sensors sensors;
    RevBlinkinLedDriver botLED;
    RevBlinkinLedDriver ballStorageLED;
    Flywheel flywheel;
    public Gamepad currentGamepad1 = new Gamepad();
    public Gamepad previousGamepad1 = new Gamepad();
    public ElapsedTime ledTimer = new ElapsedTime();
    public int ledStage = 0;

    int sensorLeft;
    int sensorRight;
    //    int sensorBack;
    int sensorLeftprev = 0;
    int sensorRightprev = 0;
    //    int sensorBackprev = 0;
//
    boolean uptospeed;

    //
    public LED(HardwareMap hardwareMap, Sensors sensors, Flywheel flywheel) {
        this.sensors = sensors;
        this.flywheel = flywheel;

        botLED = hardwareMap.get(RevBlinkinLedDriver.class, "LED");
        ballStorageLED = hardwareMap.get(RevBlinkinLedDriver.class, "storageLED")
    }

    public LED(HardwareMap hardwareMap) {
        botLED = hardwareMap.get(RevBlinkinLedDriver.class, "LED");
    }

//    public void ballLED() {
//        sensorFront = sensors.getFrontColor();
//        sensorMid = sensors.getMidColor();
//        sensorBack = sensors.getBackColor();
//
//        if (sensorFront != 0 && sensorFrontprev == 0) {
//            num_balls++;
//        } else if (sensorFront == 0 && sensorFrontprev != 0) {
//            num_balls--;
//        }
//
//        if (sensorMid != 0 && sensorMidprev == 0) {
//            num_balls++;
//        } else if (sensorMid == 0 && sensorMidprev != 0) {
//            num_balls--;
//        }
//        if (sensorBack != 0 && sensorBackprev == 0) {
//            num_balls++;
//        } else if (sensorBack == 0 && sensorBackprev != 0) {
//            num_balls--;
//        }
//
//        num_balls = Math.max(0, Math.min(num_balls, 3));
//
//        sensorFrontprev = sensorFront;
//        sensorMidprev = sensorMid;
//        sensorBackprev = sensorBack;
//
//        // sets the LED color
//        // RED == 0 ball
//        // BLUE == 1 ball
//        // GREEN == 2 balls
//        // YELLOW == 3 balls
//        if (num_balls == 0) {
//            botLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
//        } else if (num_balls == 1) {
//            botLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
//        } else if (num_balls == 2) {
//            botLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
//        } else {
//            botLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
//        }
//    }

    public void speedCheck() {
        uptospeed = flywheel.isReady();

        if(uptospeed) {
            botLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        } else {
            botLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }
    }

//    public void speedCheck() {
//        uptospeed = flywheel.isReady();
//
//        if (uptospeed) {
//            switch (ledStage) {
//
//                case 0:
//                    botLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
//                    ledTimer.reset();
//                    ledStage = 1;
//                    break;
//
//                case 1:
//                    if (ledTimer.seconds() > 0.5) {
//                        botLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
//                        ledTimer.reset();
//                        ledStage = 2;
//                    }
//                    break;
//
//                case 2:
//                    if (ledTimer.seconds() > 0.5) {
//                        botLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
//                        ledStage = 3;
//                    }
//                    break;
//
//                case 3:
//                    break;
//            }
//           botLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
//        } else {
//            botLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
//            ledStage = 0;
//        }
//    }

    public boolean ballCheckLED() {

        sensorLeft = sensors.getLeftColorLED();
        sensorRight = sensors.getRightColorLED();

        return (sensorLeft != 0 && sensorRight != 0);
        }

    public void ballLEDSet() {

        if (ballCheckLED()) {
            ballStorageLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        } else {
            ballStorageLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }
    }
}

