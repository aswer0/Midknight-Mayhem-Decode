package org.firstinspires.ftc.teamcode.FinalCode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Experiments.Utils.RGB;

@Config
public class Sensors {

    public DcMotorEx intakeMotor;

    // sensors
    RevColorSensorV3 midSensor1;
    RevColorSensorV3 midSensor2;
    RevColorSensorV3 backSensor1;
    RevColorSensorV3 backSensor2;

    DigitalChannel frontBreakBeam;
    DigitalChannel midBreakBeam;
//    DigitalChannel backBreakBeam;

    // sensor color values
    public RGB mid1 = new RGB();public  RGB mid2 = new RGB();
    public RGB back1 = new RGB();public RGB back2 = new RGB();

    public double back1D;
    public double back2D;
    public double mid1D;
    public double mid2D;
    public ElapsedTime timeSinceThreeBalls = new ElapsedTime();

    public Sensors(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        midSensor1 = hardwareMap.get(RevColorSensorV3.class,"mS1");
        midSensor2 = hardwareMap.get(RevColorSensorV3.class,"mS2");
        backSensor1 = hardwareMap.get(RevColorSensorV3.class,"bS1");
        backSensor2 = hardwareMap.get(RevColorSensorV3.class,"bS2");

        frontBreakBeam = hardwareMap.get(DigitalChannel.class, "fBB");
        midBreakBeam = hardwareMap.get(DigitalChannel.class, "mBB");
//        backBreakBeam = hardwareMap.get(DigitalChannel.class, "bBB");
        frontBreakBeam.setMode(DigitalChannel.Mode.INPUT);
        midBreakBeam.setMode(DigitalChannel.Mode.INPUT);
//        backBreakBeam.setMode(DigitalChannel.Mode.INPUT);
    }

    public boolean hasAllBalls() {
        return hasFrontBall() && hasMidBall() && hasBackBall();
    }
//        boolean result = hasAllBallsNoDelay();
//        if(!result) timeSinceThreeBalls.reset();
//        return timeSinceThreeBalls.milliseconds() > 100;
//    }
//    public boolean hasAllBallsNoDelay() {
//        return hasFrontBall() && hasMidBall() && hasBackBall();
//    }

    public boolean hasAnyBall() {
        return hasFrontBall() || hasMidBall() || hasBackBall();
    }
    public boolean hasFrontBall() {
        return !frontBreakBeam.getState();
    }
    public boolean hasMidBall() {
        return !midBreakBeam.getState();
    }
    public boolean hasBackBall() {
        back1D = backSensor1.getDistance(DistanceUnit.INCH);
        back2D = backSensor2.getDistance(DistanceUnit.INCH);
        // 0 -> no ball
        // 1 -> ball
        return (back1D < 1.467 || back2D < 1.467);
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
        back1D = backSensor1.getDistance(DistanceUnit.INCH);
        back2D = backSensor2.getDistance(DistanceUnit.INCH);
        // 0 -> no ball
        // 1 -> ball
        if(back1D < 1.467 || back2D < 1.467) return 1;
        return 0;
    }
}