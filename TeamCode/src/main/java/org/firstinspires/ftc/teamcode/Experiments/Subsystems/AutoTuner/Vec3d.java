package org.firstinspires.ftc.teamcode.Experiments.Subsystems.AutoTuner;

public class Vec3d {
    public double x;
    public double y;
    public double h;

    public double p;
    public double i;
    public double d;

    public Vec3d(double x, double y, double h){
        this.x = x;
        this.y = y;
        this.h = h;
    }
    public Vec3d(){
        this.x = 0;
        this.y = 0;
        this.h = 0;
    }
    public Vec3d(Vec3d vec3d){
        this.x = vec3d.x;
        this.y = vec3d.y;
        this.h = vec3d.h;
    }

    public void set_gains(double p, double i, double d){
        this.p=p;
        this.i=i;
        this.d=d;
    }
    public String stringGains(){
        return "p: " + p + " | i: " + i + " | d: " + d;
    }

    public void subtract(Vec3d vec3d){
        this.x -= vec3d.x;
        this.y -= vec3d.y;
        this.h -= vec3d.h;
    }
    public void add(Vec3d vec3d){
        this.x += vec3d.x;
        this.y += vec3d.y;
        this.h += vec3d.h;
    }
    public void set(double x, double y, double h){
        this.x = x;
        this.y = y;
        this.h = h;
    }
    public void set(Vec3d vec3d){
        this.x = vec3d.x;
        this.y = vec3d.y;
        this.h = vec3d.h;
    }
    public double get(char axis){
        if (axis == 'x'){
            return this.x;
        }
        if (axis == 'y'){
            return this.y;
        }
        return this.h;

    }
    public String toString(){
        return this.x + ", " + this.y + ", " + this.h;
    }
}
