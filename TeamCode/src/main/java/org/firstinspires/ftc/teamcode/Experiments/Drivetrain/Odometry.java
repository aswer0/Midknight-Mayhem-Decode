package org.firstinspires.ftc.teamcode.Experiments.Drivetrain;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Odometry {
    PinpointOdometry pinpoint;
    Telemetry telemetry;

    public Odometry(HardwareMap hardwareMap, Telemetry telemetry, String pinpointName){
        this.pinpoint = hardwareMap.get(PinpointOdometry.class, pinpointName);
        this.telemetry = telemetry;

        pinpoint.setEncoderResolution(PinpointOdometry.GoBildaOdometryPods.goBILDA_4_BAR_POD); //19.894
        /*
        Horizontal: 76.2mm
        Vertical: 152.4mm
        * */
        pinpoint.setOffsets(76.2, 152.4, DistanceUnit.MM);
        pinpoint.resetPosAndIMU();
    }

    public void reset(){
        pinpoint.resetPosAndIMU();
    }
    public void update(){
        pinpoint.update();
    }
    public double get_heading(){
        return pinpoint.getHeading(AngleUnit.DEGREES);
    }
    public double get_x(){
        return pinpoint.getPosX(DistanceUnit.INCH);
    }
    public double get_y(){
        return pinpoint.getPosY(DistanceUnit.INCH);
    }
}
