package org.firstinspires.ftc.teamcode.opmode.Calculate;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Distance {

    public static double[] red = {0.0 , 0.0 , 0.0};
    public static double[] blue = {0.0 , 0.0 , 0.0};
    private double[] target = {0.0 , 0.0 , 0.0};

    public double[] distance(double X, double Y, boolean is_red){
        if (is_red){target = red;}
        else{target = blue;}
        double delta_X = target[0] - X;
        double delta_Y = target[1] - Y;
        double theta = Math.atan2(delta_X, delta_Y)/ Math.PI * 180;

        double[] distances = {delta_X, delta_Y, theta};
        return distances;
    }
}
