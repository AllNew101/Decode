package org.firstinspires.ftc.teamcode.opmode.Calculate;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class Distance {

    public static double[] red = {128.80923076923077 , -140.4923076923077 , 33.0};
    public static double[] blue = {122, -12 , 50.0};

    private double[] target = {0.0 , 0.0 , 0.0};

    public double[] distance(double X, double Y, boolean is_red){
        if (is_red){target = red;}
        else{target = blue;}
        double delta_X = target[0] - X;
        double delta_Y = Y - target[1];
        double displacement = Math.sqrt(Math.pow(delta_X,2) + Math.pow(delta_Y,2));
        double theta = Math.atan2(delta_Y,delta_X) / Math.PI * 180;
        double[] distances = {delta_X, delta_Y, theta, displacement};

        return distances;
    }

    public double targeting(double X, double Y, boolean is_red , double theta, double offset, double limit){
        double stabilizer = theta;
        double targeting = distance(X ,Y ,is_red)[2];
        double result = targeting + stabilizer + offset;

        if (result > limit){result = limit;}
        else if (result < -limit){result = -limit;}
        return AngleUnit.normalizeDegrees(result);
    }
}
