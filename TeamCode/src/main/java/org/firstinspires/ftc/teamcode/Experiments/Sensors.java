package org.firstinspires.ftc.teamcode.Experiments;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Sensors {

    // sensors
    RevColorSensorV3 frontSensor1;
    RevColorSensorV3 frontSensor2;
    RevColorSensorV3 frontSensor3;
    RevColorSensorV3 leftSensor1;
    RevColorSensorV3 leftSensor2;
    RevColorSensorV3 rightSensor1;
    RevColorSensorV3 rightSensor2;

    // sensor color values
    double fS1_red;
    double fS1_green;
    double fS1_blue;
    double fS2_red;
    double fS2_green;
    double fS2_blue;
    double fS3_red;
    double fS3_green;
    double fS3_blue;
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


    public Sensors(HardwareMap hardwareMap) {
//        frontSensor1 = hardwareMap.get(RevColorSensorV3.class,"fS1");
//        frontSensor2 = hardwareMap.get(RevColorSensorV3.class,"fS2");
//        frontSensor3 = hardwareMap.get(RevColorSensorV3.class,"fS3");
//        leftSensor1 = hardwareMap.get(RevColorSensorV3.class,"lS1");
//        leftSensor2 = hardwareMap.get(RevColorSensorV3.class,"lS2");
//        rightSensor1 = hardwareMap.get(RevColorSensorV3.class,"rS1");
//        rightSensor2 = hardwareMap.get(RevColorSensorV3.class,"rS2");
    }

    public int getFrontColor() {

        fS1_red = frontSensor1.red();
        fS1_green = frontSensor1.green();
        fS1_blue = frontSensor1.blue();

        fS2_red = frontSensor2.red();
        fS2_green = frontSensor2.green();
        fS2_blue = frontSensor2.blue();

        fS3_red = frontSensor3.red();
        fS3_green = frontSensor3.green();
        fS3_blue = frontSensor3.blue();

        // 0 is no ball
        // 1 is purple
        // 2 is green
        if (fS1_red + fS1_green + fS1_blue >= 900) {
            if (fS1_blue > fS1_red && fS1_blue > fS1_green) {
                return 1;
            } else if (fS1_green > fS1_blue && fS1_green > fS1_red) {
                return 2;
            }
        } else if (fS2_red + fS2_green + fS2_blue >= 900) {
            if (fS2_blue > fS2_red && fS2_blue > fS2_green) {
                return 1;
            } else if (fS2_green > fS2_blue && fS2_green > fS2_red) {
                return 2;
            }
        } else if (fS3_red + fS3_green + fS3_blue >= 900) {
            if (fS3_blue > fS3_red && fS3_blue > fS3_green) {
                return 1;
            } else if (fS3_green > fS3_red && fS3_green > fS3_blue) {
                return 2;
            }
        }
        return 0;
    }

    public int getLeftColor() {

        lS1_red = leftSensor1.red();
        lS1_green = leftSensor1.green();
        lS1_blue = leftSensor1.blue();

        lS2_red = leftSensor2.red();
        lS2_green = leftSensor2.green();
        lS2_blue = leftSensor2.blue();

        // 0 is no ball
        // 1 is purple
        // 2 is green
        if (lS1_red + lS1_green + lS1_blue >= 900) {
            if (lS1_blue > lS1_red && lS1_blue > lS1_green) {
                return 1;
            } else if (lS1_green > lS1_blue && lS1_green > lS1_red) {
                return 2;
            }
        } else if (lS2_red + lS2_green + lS2_blue >= 900) {
            if (lS2_blue > lS2_red && lS2_blue > lS2_green) {
                return 1;
            } else if (lS2_green > lS2_blue && lS2_green > lS2_red) {
                return 2;
            }
        }
        return 0;
    }

    public int getRightColor() {

        rS1_red = rightSensor1.red();
        rS1_green = rightSensor1.green();
        rS1_blue = rightSensor1.blue();

        rS2_red = rightSensor2.red();
        rS2_green = rightSensor2.green();
        rS2_blue = rightSensor2.blue();

        // 0 is no ball
        // 1 is purple
        // 2 is green
        if (rS1_red + rS1_green + rS1_blue >= 900) {
            if (rS1_blue > rS1_red && rS1_blue > rS1_green) {
                return 1;
            } else if (rS1_green > rS1_blue && rS1_green > rS1_red) {
                return 2;
            }
        } else if (rS2_red + rS2_green + rS2_blue >= 900) {
            if (rS2_blue > rS2_red && rS2_blue > rS2_green) {
                return 1;
            } else if (rS2_green > rS2_blue && rS2_green > rS2_red) {
                return 2;
            }
        }
        return 0;
    }
}
