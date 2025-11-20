package org.firstinspires.ftc.teamcode.Experiments.Subsystems.AutoTuner;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Drivetrain.GVF.BCPath;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Drivetrain.GVF.VectorField;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Drivetrain.WheelControl;
import org.opencv.core.Point;

public class ZieglerNichols{
    public double Ku;
    public double Pu;
    public double Ti;
    public double Td;

    Vec3d gains;

    /*
    set_params to put ultimate gain Ku and period of oscillation Pu
     */

    public void set_params(double Ku, double Pu){
        this.Ku = Ku;
        this.Pu = Pu;
    }
    public Vec3d Update(){
        double kp = 0.6*Ku;
        Ti = Pu/1.2;
        Td = Pu/8;

        gains.set_gains(kp, kp/Ti, kp*Td);
        return gains;
    }
    public void debug(TelemetryPacket packet, String axis){
        packet.put(axis + " gains", gains.stringGains());
    }

}
