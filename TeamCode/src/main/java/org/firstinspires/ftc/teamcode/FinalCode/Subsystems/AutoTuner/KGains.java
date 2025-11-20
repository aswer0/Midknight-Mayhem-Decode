package org.firstinspires.ftc.teamcode.FinalCode.Subsystems.AutoTuner;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class KGains {
    public double xp, yp, hp;
    public double xd, yd, hd;

    public double eta_d;
    public double eta_p;

    Telemetry telemetry;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    public KGains(Telemetry telemetry, Vec3d kp, Vec3d kd, double eta_p, double eta_d){
        xp = kp.x;
        yp = kp.y;
        hp = kp.h;

        xd = kd.x;
        yd = kd.y;
        hd = kd.h;

        this.eta_d = eta_d;
        this.eta_p = eta_p;

        this.telemetry = telemetry;
    }

    public Vec3d get_p_gains(){
        return new Vec3d(xp, yp, hp);
    }
    public Vec3d get_d_gains(){
        return new Vec3d(xd, yd, hd);
    }

    public double tanh(double k, double v){
        return Math.tanh(k*v);
    }

    public void Update(Vec3d error){
        this.xp = this.xp + eta_p*tanh(0.5, error.x);
        this.yp = this.yp + eta_p*tanh(0.5, error.y);
        this.hp = this.hp + eta_p*tanh(0.5, error.h);

        this.xd = this.xd - eta_p*tanh(0.5, error.x);
        this.yd = this.yd - eta_p*tanh(0.5, error.y);
        this.hd = this.hd - eta_p*tanh(0.5, error.h);
    }

    public void Update(Vec3d error, Vec3d O){
        Vec3d loss = new Vec3d(error);
        loss.subtract(O);

        this.xp = this.xp + eta_p*tanh(0.5, loss.x);
        this.yp = this.yp + eta_p*tanh(0.5, loss.y);
        this.hp = this.hp + eta_p*tanh(0.5, loss.h);

        this.xd = this.xd - eta_p*tanh(0.5, loss.x);
        this.yd = this.yd - eta_p*tanh(0.5, loss.y);
        this.hd = this.hd - eta_p*tanh(0.5, loss.h);
    }
    public void debug(){
        telemetry.addData("Proportional x", this.xp);
        telemetry.addData("Proportional y", this.yp);
        telemetry.addData("Proportional h", this.hp);

        telemetry.addData("Derivative x", this.xd);
        telemetry.addData("Derivative y", this.yd);
        telemetry.addData("Derivative h", this.hd);



        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Proportional x", this.xp);
        packet.put("Proportional y", this.yp);
        packet.put("Proportional h", this.hp);

        packet.put("Derivative x", this.xd);
        packet.put("Derivative y", this.yd);
        packet.put("Derivative h", this.hd);

        dashboard.sendTelemetryPacket(packet);
    }

}
