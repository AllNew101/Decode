package org.firstinspires.ftc.teamcode.opmode.system;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class angular_set {
    private Servo angle;
    public double speed_angular = 100;


    private TelemetryManager telemetryM;

    public void init_angular(HardwareMap hardwareMap) {
        angle = hardwareMap.get(Servo.class, "angle_2");
    }

    public double angular(double speed_angular, boolean is_get_position, double angular_set){
        double angularx;
        if (is_get_position){
            angularx = angle.getPosition() + (speed_angular / 200);
            angle.setPosition(angularx);
        }
        else{
            angularx = angular_set + (speed_angular / 200);
            angle.setPosition(angularx);
        }
        return angularx;
    }
    public void set_offset(double position){
        angle.setPosition(angle.getPosition() + position);
    }

    public void set_position(double position){
        angle.setPosition(position);
    }

    public void setup_servo(){
        angle.setPosition(0.5);
    }

}
