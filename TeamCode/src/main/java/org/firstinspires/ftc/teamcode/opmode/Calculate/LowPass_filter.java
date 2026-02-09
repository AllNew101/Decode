package org.firstinspires.ftc.teamcode.opmode.Calculate;

public class LowPass_filter {
    double omegaFiltered = 0;
    double alpha;
    public LowPass_filter(double alpha){

    }
    public double filter(double omegaRaw){
        omegaFiltered = alpha * omegaFiltered + (1 - alpha) * omegaRaw;
        return omegaFiltered;
    }
}
