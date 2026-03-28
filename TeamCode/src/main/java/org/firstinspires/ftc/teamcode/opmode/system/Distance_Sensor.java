package org.firstinspires.ftc.teamcode.opmode.system;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Distance_Sensor {
    public Servo LED;
    public DistanceSensor Dis_sen;

    public static double not_ready = 0.28;
    public static double ready = 0.5;
    public static double test = 14;
    double current ;
    double previos;
    public static double time_delay = 0.35;
    boolean error = false;
    ElapsedTime Time;



    public void init_Distance_senser (HardwareMap hardwareMap , ElapsedTime time) {
        Dis_sen = hardwareMap.get(DistanceSensor.class,"dis_sen");
        LED = hardwareMap.get(Servo.class , "LED");
        Time = time;
        previos = Time.seconds();
    }

    public double get_dis() {
        double distance = Dis_sen.getDistance(DistanceUnit.CM);
        return distance;
    }

    public void check_led() {
        current = Time.seconds() - previos;
        if (get_dis() < test && current > time_delay){
            LED.setPosition(ready);
        }
        else if (get_dis() > test ) {
            LED.setPosition(not_ready);
            previos = Time.seconds();
        }

    }
}
