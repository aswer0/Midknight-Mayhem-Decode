package org.firstinspires.ftc.teamcode.FinalCode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Sensors {

    // sensors
    RevColorSensorV3 frontSensor1;
    RevColorSensorV3 frontSensor2;
    RevColorSensorV3 leftSensor1;
    RevColorSensorV3 leftSensor2;
    RevColorSensorV3 rightSensor1;
    RevColorSensorV3 rightSensor2;

    // sensors for LED
    RevColorSensorV3 leftLEDSensor;
    RevColorSensorV3 rightLEDSensor;

    // sensor color values
    double fS1_red;
    double fS1_green;
    double fS1_blue;
    double fS2_red;
    double fS2_green;
    double fS2_blue;
    double lS1_red;
    double lS1_green;
    double lS1_blue;
    double lS2_red;
    double lS2_green;
    double lS2_blue;
    double rS1_red;
    double rS1_green;
    double rS1_blue;
    double rS2_red;
    double rS2_green;
    double rS2_blue;

    // sensor values for LED
    double l_red;
    double l_green;
    double l_blue;
    double r_red;
    double r_green;
    double r_blue;

    public Sensors(HardwareMap hardwareMap) {
//        frontSensor1 = hardwareMap.get(RevColorSensorV3.class,"fS1");
//        frontSensor2 = hardwareMap.get(RevColorSensorV3.class,"fS2");
//        frontSensor3 = hardwareMap.get(RevColorSensorV3.class,"fS3");
//        leftSensor1 = hardwareMap.get(RevColorSensorV3.class,"lS1");
//        leftSensor2 = hardwareMap.get(RevColorSensorV3.class,"lS2");
//        rightSensor1 = hardwareMap.get(RevColorSensorV3.class,"rS1");
//        rightSensor2 = hardwareMap.get(RevColorSensorV3.class,"rS2");
        leftLEDSensor = hardwareMap.get(RevColorSensorV3.class, "LeftSensor");
        rightLEDSensor = hardwareMap.get(RevColorSensorV3.class, "RightSensor");
    }

    public int getFrontColor() {

        fS1_red = frontSensor1.red();
        fS1_green = frontSensor1.green();
        fS1_blue = frontSensor1.blue();

        // 0 -> no ball
        // 1 -> ball
        if (fS1_red + fS1_green + fS1_blue >= 900) {
            return 1;
        } else {
            fS2_red = frontSensor2.red();
            fS2_green = frontSensor2.green();
            fS2_blue = frontSensor2.blue();

            if (fS2_red + fS2_green + fS2_blue >= 900) {
                return 1;
            } else {
                return 0;
            }
        }
    }

    public int getMidColor() {

        lS1_red = leftSensor1.red();
        lS1_green = leftSensor1.green();
        lS1_blue = leftSensor1.blue();

        // 0 -> no ball
        // 1 -> ball
        if (lS1_red + lS1_green + lS1_blue >= 900) {
            return 1;
        } else {
            lS2_red = leftSensor2.red();
            lS2_green = leftSensor2.green();
            lS2_blue = leftSensor2.blue();

            if (lS2_red + lS2_green + lS2_blue >= 900) {
                return 1;
            } else {
                return 0;
            }
        }
    }

    public int getBackColor() {

        rS1_red = rightSensor1.red();
        rS1_green = rightSensor1.green();
        rS1_blue = rightSensor1.blue();

        // 0 -> no ball
        // 1 -> ball
        if (rS1_red + rS1_green + rS1_blue >= 900) {
            return 1;
        } else {
            rS2_red = rightSensor2.red();
            rS2_green = rightSensor2.green();
            rS2_blue = rightSensor2.blue();

            if (rS2_red + rS2_green + rS2_blue >= 900) {
                return 1;
            } else {
                return 0;
            }
        }
    }

    public int getLeftColorLED() {

        l_red = leftLEDSensor.red();
        l_green = leftLEDSensor.green();
        l_blue = leftLEDSensor.blue();

        // 0 is no ball
        // 1 purple
        // 2 is green
        if (l_red + l_green + l_blue >= 900) {
            if (l_blue > l_red && l_blue > l_green) {
                return 1;
            } else if (l_green > l_blue && l_green > l_red) {
                return 2;
            }
        }
        return 0;
    }

    public int getRightColorLED() {

        r_red = rightLEDSensor.red();
        r_green = rightLEDSensor.green();
        r_blue = rightLEDSensor.blue();

        // 0 is no ball
        // 1 purple
        // 2 is green
        if (r_red + r_green + r_blue >= 900) {
            if (r_blue > r_red && r_blue > r_green) {
                return 1;
            } else if (r_green > r_blue && r_green > r_red) {
                return 2;
            }
        }
        return 0;
    }
}