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
    public DistanceSensor Front_Dissen;
    public DistanceSensor Center_Dissen;

    public double not_ready = 0.28;
    public double ready = 0.5;;
    ElapsedTime Time;



    public void init_Distance_senser (HardwareMap hardwareMap) {
        Front_Dissen = hardwareMap.get(DistanceSensor.class,"dis_sen");
        Center_Dissen = hardwareMap.get(DistanceSensor.class,"Dis_sen2");
        LED = hardwareMap.get(Servo.class , "LED");

    }

    public double get_Front_dis() {
        double distance = Front_Dissen.getDistance(DistanceUnit.CM);
        return distance;
    }
    public double get_Center_dis() {
        double distance = Center_Dissen.getDistance(DistanceUnit.CM);
        return distance;
    }
    public void check_led() {
        if (get_Front_dis() <= 14 && get_Center_dis() <= 14){
            LED.setPosition(ready);
        }
        else{
            LED.setPosition(not_ready);
        }
    }
    public boolean led_status (){
            return (LED.getPosition() == ready);
    }
}
