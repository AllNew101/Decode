package org.firstinspires.ftc.teamcode.opmode.Calculate;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
public class LowPass_filter {
    double omegaFiltered = 0;
    double alpha;
    public LowPass_filter(double alpha1){
        this.alpha = alpha1;
    }
    public double filter(double omegaRaw){
        omegaFiltered = alpha * omegaFiltered + (1 - alpha) * omegaRaw;
        return omegaFiltered;
    }
}
