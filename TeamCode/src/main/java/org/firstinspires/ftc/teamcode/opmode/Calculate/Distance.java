package org.firstinspires.ftc.teamcode.opmode.Calculate;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Distance {

    public static double[] red = {138.80923076923077 , -134.4923076923077 , 33.0};
    public static double[] blue = {122, -12 , 50.0};
    public static double coff = 27;
    public static double increase_coff = 0;
    private double[] target = {0.0 , 0.0 , 0.0};

    public double[] distance(double X, double Y, boolean is_red){
        if (is_red){target = red;}
        else{target = blue;}
        double delta_X = target[0] - X;
        double delta_Y = Y - target[1];
        double displacement = Math.sqrt(Math.pow(delta_X,2) + Math.pow(delta_Y,2));
        double theta = Math.atan2(delta_X, delta_Y) / Math.PI * 180;
        double[] distances = {delta_X, delta_Y, theta, displacement};

        return distances;
    }

    public double targeting(double X, double Y, boolean is_red , double theta, double offset, double limit){
        double angle;
        if (is_red){angle = red[2];}
        else{angle = blue[2];}
        if (theta < 0){theta = 360 + theta;}
        double result = (angle - 180  + theta) - (coff - (90 - distance(X ,Y ,is_red)[2]))  + offset;
       // if (theta > 170){result += increase_coff;}

        if(result > limit){return limit;}
        else if(result < -limit){return -limit;}
        return result;
    }
}
