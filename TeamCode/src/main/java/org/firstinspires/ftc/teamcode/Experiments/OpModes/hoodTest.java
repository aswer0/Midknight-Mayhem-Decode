package org.firstinspires.ftc.teamcode.Experiments.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Outtake.Hood;

@Config
@TeleOp
public class hoodTest extends OpMode {
    public static double angle = 0;
    public static double tick = 0;
    public static Hood hood;

    public static boolean settingTicks = true;

    @Override
    public void init() {
        hood = new Hood(hardwareMap);
    }

    @Override
    public void loop() {

        if (settingTicks){
            hood.set_tick(tick);
        }
        else{
            hood.set_angle(angle);
        }
    }
}
