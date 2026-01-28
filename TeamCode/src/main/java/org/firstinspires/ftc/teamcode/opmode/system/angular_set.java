package org.firstinspires.ftc.teamcode.opmode.system;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//Don't have angular now.
@Config
public class angular_set {
    private Servo angle;
    public double speed_angular = 100;
    public static int divided_COF = 800;
    public void init_angular(HardwareMap hardwareMap) {
        angle = hardwareMap.get(Servo.class, "angle_2");
    }

    public void angular_on(double speed_angular){
        double angularx;
        angularx = angle.getPosition() + (speed_angular / divided_COF);
        angle.setPosition(angularx);
    }

    public double angular_on(double speed_angular , double angular){
        double angularx;
        angularx = angular + (speed_angular / divided_COF);
        angle.setPosition(angularx);
        return angularx;
    }

    public void angular_to_position(double speed_angular , double position){
        double angularx;
        int indicator = 1;
        if (angle.getPosition() < position){indicator = -1;}
        if (angle.getPosition() != position){angularx =  + (speed_angular / divided_COF) * indicator;}
        angularx = angle.getPosition() + (speed_angular / divided_COF) * indicator;
        angle.setPosition(angularx);
    }
    public double angular_to_position(double speed_angular , double angular , double position){
        double angularx = angular;
        int indicator = 1;
        if (angular < position){indicator = -1;}
        if (angular != position){angularx = angular + (speed_angular / divided_COF) * indicator;}
        angle.setPosition(angularx);
        return angularx;
    }

    public void set_offset(double position){
        angle.setPosition(angle.getPosition() + position);
    }

    public void setPosition(double position){
        angle.setPosition(position);
    }

    public void setup_servo(){
        angle.setPosition(0.5);
    }

    public double get_position(){
        return angle.getPosition();
    }

    public void short_cut(int key){
        switch (key){
            case 0:
                angle.setPosition(0.38);
            case 1:
                angle.setPosition(0.56);
        }
    }

}
