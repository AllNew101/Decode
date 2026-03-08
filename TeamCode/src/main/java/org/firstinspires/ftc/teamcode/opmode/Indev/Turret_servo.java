package org.firstinspires.ftc.teamcode.opmode.Indev;
import com.acmerobotics.dashboard.config.Config;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Turret_servo {
    private Servo Turret;
    private Servo Turret2;
    double set_speed_drifter = 0;
    double old_target = 0.0;
    public void init_Turret_servo (HardwareMap hardwareMap) {
        Turret = hardwareMap.get(Servo.class, "Turret-0");
        Turret2 = hardwareMap.get(Servo.class, "Turret-1");
    }
    public void Turret_ServoON(double set_speed){
        double current_speed;
        set_speed_drifter = set_speed;
        current_speed = Turret.getPosition() + set_speed;
        Turret.setPosition(current_speed);
        Turret2.setPosition(current_speed);
    }

    public double get_angle(){
        return  46.5454 - (93.0909 * (Turret.getPosition()));
    }
    public void set_Position(double position){
        Turret.setPosition(position);
        Turret2.setPosition(position);
    }

    public double get_Position(){return Turret.getPosition();}
    public double get_power(){return set_speed_drifter;}

    public void to_position(double angle){
        double position = (46.5454 - angle) / 93.0909;
        Turret.setPosition(position);
        Turret2.setPosition(position);
    }
    public double get_exact_YAW(double theta){
        return  get_angle() + theta;
    }



    public void stop(){
        Turret.setPosition(Turret.getPosition());
        Turret2.setPosition(Turret.getPosition());
    }

}
