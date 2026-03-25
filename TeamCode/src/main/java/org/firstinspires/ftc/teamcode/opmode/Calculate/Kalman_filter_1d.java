package org.firstinspires.ftc.teamcode.opmode.Calculate;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
public class Kalman_filter_1d {
    private double omega;

    private double p; // Covariance

    private double q;
    private double r;

    public Kalman_filter_1d(double q, double r){
        this.q = q;
        this.r = r;

        omega = 0.0;
        p = 100.0;
    }
    public void update(double measure_omega){
        // Predict
        double Predict_Omega = omega;
        double Predict_P = p + q;

        //Estimate
        double k = Predict_P / (Predict_P + r);
        omega = Predict_Omega + k * (measure_omega - Predict_Omega);
        p = (1 - k) * Predict_P;
    }
    public double get_Omega(){return omega;}


}
