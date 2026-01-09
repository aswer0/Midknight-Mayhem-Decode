package org.firstinspires.ftc.teamcode.FinalCode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Experiments.Utils.RGB;

@Config
public class Sensors {

    // sensors
    RevColorSensorV3 frontSensor1;
    RevColorSensorV3 frontSensor2;
    RevColorSensorV3 midSensor1;
    RevColorSensorV3 midSensor2;
    RevColorSensorV3 backSensor1;
    RevColorSensorV3 backSensor2;

    // sensors for LED
    RevColorSensorV3 leftLEDSensor;
    RevColorSensorV3 rightLEDSensor;

    RGB rgb;

    // sensor color values
    RGB front1 = new RGB(); RGB front2 = new RGB();
    RGB mid1; RGB mid2;
    RGB back1; RGB back2;

    // sensor values for LED
    RGB left_LED;
    RGB right_LED;

    public Sensors(HardwareMap hardwareMap) {
        frontSensor1 = hardwareMap.get(RevColorSensorV3.class,"LeftSensor");
        frontSensor2 = hardwareMap.get(RevColorSensorV3.class,"RightSensor");
//        midSensor1 = hardwareMap.get(RevColorSensorV3.class,"fS3");
//        midSensor2 = hardwareMap.get(RevColorSensorV3.class,"mS1");
//        backSensor1 = hardwareMap.get(RevColorSensorV3.class,"mS2");
//        backSensor2 = hardwareMap.get(RevColorSensorV3.class,"bS1");
//
//        leftLEDSensor = hardwareMap.get(RevColorSensorV3.class, "LeftSensor");
//        rightLEDSensor = hardwareMap.get(RevColorSensorV3.class, "RightSensor");
    }

    public int getFrontColor() {
        front1.set(
            frontSensor1.red(),
            frontSensor1.green(),
            frontSensor1.blue()
        );
        front2.set(
                frontSensor2.red(),
                frontSensor2.green(),
                frontSensor2.blue()
        );

        // 0 -> no ball
        // 1 -> ball
        if (front1.r + front1.g + front1.b >= 900) {
            return 1;
        } else {
            if (front2.r + front2.g + front2.b >= 900) {
                return 1;
            } else {
                return 0;
            }
        }
    }

//    public int getMidColor() {
//
//        lS1_red = leftSensor1.red();
//        lS1_green = leftSensor1.green();
//        lS1_blue = leftSensor1.blue();
//
//        // 0 -> no ball
//        // 1 -> ball
//        if (lS1_red + lS1_green + lS1_blue >= 900) {
//            return 1;
//        } else {
//            lS2_red = leftSensor2.red();
//            lS2_green = leftSensor2.green();
//            lS2_blue = leftSensor2.blue();
//
//            if (lS2_red + lS2_green + lS2_blue >= 900) {
//                return 1;
//            } else {
//                return 0;
//            }
//        }
//    }
//
//    public int getBackColor() {
//
//        rS1_red = rightSensor1.red();
//        rS1_green = rightSensor1.green();
//        rS1_blue = rightSensor1.blue();
//
//        // 0 -> no ball
//        // 1 -> ball
//        if (rS1_red + rS1_green + rS1_blue >= 900) {
//            return 1;
//        } else {
//            rS2_red = rightSensor2.red();
//            rS2_green = rightSensor2.green();
//            rS2_blue = rightSensor2.blue();
//
//            if (rS2_red + rS2_green + rS2_blue >= 900) {
//                return 1;
//            } else {
//                return 0;
//            }
//        }
//    }
//
//    public int getLeftColorLED() {
//
//        l_red = leftLEDSensor.red();
//        l_green = leftLEDSensor.green();
//        l_blue = leftLEDSensor.blue();
//
//        // 0 is no ball
//        // 1 purple
//        // 2 is green
//        if (l_red + l_green + l_blue >= 900) {
//            if (l_blue > l_red && l_blue > l_green) {
//                return 1;
//            } else if (l_green > l_blue && l_green > l_red) {
//                return 2;
//            }
//        }
//        return 0;
//    }
//
//    public int getRightColorLED() {
//
//        r_red = rightLEDSensor.red();
//        r_green = rightLEDSensor.green();
//        r_blue = rightLEDSensor.blue();
//
//        // 0 is no ball
//        // 1 purple
//        // 2 is green
//        if (r_red + r_green + r_blue >= 900) {
//            if (r_blue > r_red && r_blue > r_green) {
//                return 1;
//            } else if (r_green > r_blue && r_green > r_red) {
//                return 2;
//            }
//        }
//        return 0;
//    }
}