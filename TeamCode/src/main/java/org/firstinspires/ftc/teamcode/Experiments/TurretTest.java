package org.firstinspires.ftc.teamcode.Experiments;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake.Turret;

@TeleOp
@Config
public class TurretTest extends OpMode {
    Turret turret;
    public static double angle=0;

    @Override
    public void init() {
        turret = new Turret(hardwareMap, true);
    }

    @Override
    public void loop() {
        turret.setAngle(angle);

        telemetry.update();
        turret.update();
        telemetry.addData("ticks", turret.getTicks());
        telemetry.addData("angle", turret.getAngle());
    }
}
