package org.firstinspires.ftc.teamcode.FinalCode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Experiments.Utils.RGB;

@Config
public class Sensors {

    public DcMotorEx intakeMotor;
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
    public RGB front1 = new RGB();public  RGB front2 = new RGB();
    public RGB mid1 = new RGB();public  RGB mid2 = new RGB();
    public RGB back1 = new RGB();public RGB back2 = new RGB();

    public double back1D;
    public double back2D;
    public double mid1D;
    public double mid2D;

    // sensor values for LED
    RGB left_LED;
    RGB right_LED;

    public Sensors(HardwareMap hardwareMap) {
//        frontSensor1 = hardwareMap.get(RevColorSensorV3.class,"LeftSensor");
//        frontSensor2 = hardwareMap.get(RevColorSensorV3.class,"RightSensor");
//        midSensor1 = hardwareMap.get(RevColorSensorV3.class, "MidSensor1");
//        midSensor2 = hardwareMap.get(RevColorSensorV3.class, "MidSensor2");
//        backSensor1 = hardwareMap.get(RevColorSensorV3.class, "BackSensor1");
//        backSensor2 = hardwareMap.get(RevColorSensorV3.class, "BackSensor2");
        frontSensor1 = hardwareMap.get(RevColorSensorV3.class,"fS1");
        frontSensor2 = hardwareMap.get(RevColorSensorV3.class,"fS2");
        midSensor1 = hardwareMap.get(RevColorSensorV3.class,"mS1");
        midSensor2 = hardwareMap.get(RevColorSensorV3.class,"mS2");
        backSensor1 = hardwareMap.get(RevColorSensorV3.class,"bS1");
        backSensor2 = hardwareMap.get(RevColorSensorV3.class,"bS2");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

//        leftLEDSensor = hardwareMap.get(RevColorSensorV3.class, "LeftSensor");
//        rightLEDSensor = hardwareMap.get(RevColorSensorV3.class, "RightSensor");
    }

    public int getFrontColor() {
        if(intakeMotor.getCurrent(CurrentUnit.AMPS) > 6.7) return 1;
        else return 0;
//        front1.set(
//            frontSensor1.red(),
//            frontSensor1.green(),
//            frontSensor1.blue()
//        );
//        front2.set(
//                frontSensor2.red(),
//                frontSensor2.green(),
//                frontSensor2.blue()
//        );
//
//        // 0 -> no ball
//        // 1 -> ball
//        if (front1.r + front1.g + front1.b >= 900) {
//            return 1;
//        } else {
//            if (front2.r + front2.g + front2.b >= 900) {
//                return 1;
//            } else {
//                return 0;
//            }
//        }
    }

    public int getMidColor() {
        mid1.set(
            midSensor1.red(),
            midSensor1.green(),
            midSensor1.blue()
        );
        mid2.set(
          midSensor2.red(),
          midSensor2.green(),
          midSensor2.blue()
        );
        mid1D = midSensor1.getDistance(DistanceUnit.INCH);
        mid2D = midSensor2.getDistance(DistanceUnit.INCH);
        // 0 -> no ball
        // 1 -> ball
        if (mid1.r + mid1.g + mid1.b >= 900) {
            return 1;
        } else {
            if (mid2.r + mid2.g + mid2.b >= 900) {
                return 1;
            } else {
                return 0;
            }
        }
    }

    public int getBackColor() {
//        back1.set(
//                backSensor1.red(),
//                backSensor1.green(),
//                backSensor1.blue()
//        );
//        back2.set(
//                backSensor2.red(),
//                backSensor2.green(),
//                backSensor2.blue()
//        );
        back1D = backSensor1.getDistance(DistanceUnit.INCH);
        back2D = backSensor2.getDistance(DistanceUnit.INCH);
        // 0 -> no ball
        // 1 -> ball
//        if (back1.r + back1.g + back1.b >= 900) {
//            return 1;
//        } else {
//            if (back2.r + back2.g + back2.b >= 900) {
//                return 1;
//            } else {
//                return 0;
//            }
//        }
        if(back1D < 1.467 || back2D < 1.467) return 1;
        return 0;
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