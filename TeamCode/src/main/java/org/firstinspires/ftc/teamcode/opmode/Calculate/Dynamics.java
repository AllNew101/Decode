package org.firstinspires.ftc.teamcode.opmode.Calculate;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.opmode.Calculate.Distance;

public class Dynamics {
    public Pose XY_dynamics(double x_robot,double y_robot,double distance, double vball, double margin, double theta){
        double margin_vel = margin;

        if (margin > 30){margin_vel = 30;}
        double X = margin_vel * Math.sin(Math.toRadians(theta));
        double Y = margin_vel * Math.cos(Math.toRadians(theta));
        double t = distance /  vball;
        Pose pos = new Pose(x_robot + X * t,y_robot + Y * t);
        return pos;
    }


}
