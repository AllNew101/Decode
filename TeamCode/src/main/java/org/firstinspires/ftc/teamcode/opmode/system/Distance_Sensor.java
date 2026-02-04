package org.firstinspires.ftc.teamcode.opmode.system;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Distance_Sensor {
    DistanceSensor UP,DOWN;
    public static double UP_COFF = 10;
    public static double DOWN_COFF = 10;

    public void init_distance(HardwareMap hardwareMap){
        UP = hardwareMap.get(DistanceSensor.class,"UP");
        DOWN = hardwareMap.get(DistanceSensor.class,"DOWN");   }

    public boolean Is_distanceUp(){return(UP.getDistance(DistanceUnit.CM) > UP_COFF );}
    public boolean Is_distanceDown(){
        return (DOWN.getDistance(DistanceUnit.CM) > DOWN_COFF );
    }
    public boolean Is_three(){
        return (DOWN.getDistance(DistanceUnit.CM) > DOWN_COFF && UP.getDistance(DistanceUnit.CM) > UP_COFF );
    }

    public double get_distanceUp(){
        return UP.getDistance(DistanceUnit.CM);
    }
    public double get_distanceDown(){
        return DOWN.getDistance(DistanceUnit.CM);
    }

}
