package org.firstinspires.ftc.teamcode.Experiments.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Experiments.DrivetrainExperiments.Camera;
import org.firstinspires.ftc.teamcode.FinalCode.Subsystems.Outtake.Turret;

@TeleOp
@Config
public class TurretTest extends OpMode {
    Turret turret;
    public static double angle=0;

    @Override
    public void init() {

        turret = new Turret(hardwareMap, new Camera(hardwareMap), true);
    }

    @Override
    public void loop() {
        turret.setAngle(angle);

        turret.update();
    }
}
