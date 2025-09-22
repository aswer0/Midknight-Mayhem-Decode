package org.firstinspires.ftc.teamcode.Experiments;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Pinpoint.PinpointOdometry;

@TeleOp
public class pinpoint_test extends OpMode {
    PinpointOdometry pinpoint;

    @Override
    public void init(){
        pinpoint = hardwareMap.get(PinpointOdometry.class, "pinpoint");

        pinpoint.setEncoderResolution(PinpointOdometry.GoBildaOdometryPods.goBILDA_4_BAR_POD); //19.894
        /*
        Horizontal: 76.2mm
        Vertical: 152.4mm
        * */
        pinpoint.setOffsets(76.2, 152.4, DistanceUnit.MM);
        pinpoint.resetPosAndIMU();
    }

    @Override
    public void loop(){

    }
}
