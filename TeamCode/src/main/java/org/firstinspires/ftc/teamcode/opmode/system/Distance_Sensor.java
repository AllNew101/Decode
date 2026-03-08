package org.firstinspires.ftc.teamcode.opmode.system;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Distance_Sensor {
    public Servo LED;
    public DistanceSensor Dis_sen;

    public static double not_ready = 0.28;
    public static double ready = 0.5;
    public static double test = 14;



    public void init_Distance_senser (HardwareMap hardwareMap) {
        Dis_sen = hardwareMap.get(DistanceSensor.class,"dis_sen");
        LED = hardwareMap.get(Servo.class , "LED");
    }

    public double get_dis() {return Dis_sen.getDistance(DistanceUnit.CM);}

    public void check_led() {
        if (get_dis() < test) {LED.setPosition(ready);}
        else {LED.setPosition(not_ready);}
    }
}
