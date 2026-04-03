package org.firstinspires.ftc.teamcode.opmode.Calculate;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.opmode.Calculate.Distance;

public class Dynamics {
    public double XY_dynamics(double margin ,double theta){
        double X = margin * Math.sin(Math.toRadians(theta + 37));
        double Y = margin * Math.cos(Math.toRadians(theta + 37));

        return 0;
    }


}
