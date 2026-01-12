package org.firstinspires.ftc.teamcode.Experiments.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Sensors;

@TeleOp
@Config
public class inakeTest extends OpMode {

    Intake intake;
    Sensors sensors;

    public static boolean DOOR_UP = true;

    @Override
    public void init() {
        sensors = new Sensors(hardwareMap);
        intake = new Intake(hardwareMap, sensors);
    }

    @Override
    public void loop() {
        if (DOOR_UP){
            intake.doorOpen();
        }
        else{
            intake.doorClose();
        }
        intake.colorSensor();
    }
}
