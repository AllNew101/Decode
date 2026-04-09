package org.firstinspires.ftc.teamcode.opmode.Calculate;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.opmode.system.Turret;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class Distance {

    public static double[] red = {128.80923076923077 , -140.4923076923077 , 33.0};
    public static double[] blue = {123, -18 , 50.0};

    private double[] target = {0.0 , 0.0 , 0.0};
    private Turret turret;
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

    public double targeting(double X, double Y, boolean is_red , double theta, double offset, double limit, int mode){
        double stabilizer = theta;
        double targeting = distance(X ,Y ,is_red)[2];
        double result = targeting + stabilizer + offset;

            //Red
        if (Math.abs(result) > limit && is_red) {
                result = -limit;
        }
            //Blue
        else if (Math.abs(result) > limit && !is_red) {
                result = limit;
        }


        return AngleUnit.normalizeDegrees(result);
    }
}
