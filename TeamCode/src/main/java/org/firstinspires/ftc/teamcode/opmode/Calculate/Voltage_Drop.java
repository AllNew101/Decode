package org.firstinspires.ftc.teamcode.opmode.Calculate;

import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Voltage_Drop {
    VoltageSensor voltage;
    public static double Voltage_threshold = 10.00;
    public static double Normal_Voltage = 12.8;

    public void init_Voltage(HardwareMap hardwareMap){
        voltage = hardwareMap.voltageSensor.iterator().next();
    }
    public double[] Voltage_checker() {
        double status = 0;
        double scale = voltage.getVoltage() / Normal_Voltage * 0.7;
        if (voltage.getVoltage() < Voltage_threshold){status = 1;}
        else {status = 0;}
        double[] x = {status, voltage.getVoltage(), scale};
        return x;
    }
}
