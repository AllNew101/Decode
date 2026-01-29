package org.firstinspires.ftc.teamcode.opmode.Calculate;

public class Interpolation {
    public double interpolation(double d, double x1, double x2, double y1, double y2){
        return y1 + (y2 - y1) / (x2 -x1) * (d - x1);
    }
}
