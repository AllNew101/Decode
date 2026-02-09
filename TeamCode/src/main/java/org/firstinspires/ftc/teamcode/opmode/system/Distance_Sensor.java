package org.firstinspires.ftc.teamcode.opmode.system;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Distance_Sensor {
    DistanceSensor UP,DOWN;
    public static double DOWN_COFF = 12;

    int count = 0;
    int time = 0;
    public void init_distance(HardwareMap hardwareMap){
        DOWN = hardwareMap.get(DistanceSensor.class,"DOWN");
    }

    public boolean Is_distanceDown(){
        return (DOWN.getDistance(DistanceUnit.CM) < DOWN_COFF );
    }
    public boolean Is_three(){
        if (count > 300 && Is_distanceDown()){
            count = 0;
            return true;}
        else {
            count++;
            return false;}
    }

    public boolean check_time(){
        if (Is_distanceDown()){
            time = 0;
            return false;
        } else if (time > 13) {
            return true;
        }else{
            time++;
            return false;
        }
    }

    public double get_distanceDown(){
        return DOWN.getDistance(DistanceUnit.CM);
    }

}
