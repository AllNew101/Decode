package org.firstinspires.ftc.teamcode.opmode.Indev;
import com.acmerobotics.dashboard.config.Config;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Turret_servo {
    private Servo Turret;
    public static double constant = 0.09;
    double set_speed_drifter = 0;
    double old_target = 0.0;
    public void init_Turret_servo (HardwareMap hardwareMap) {
        Turret = hardwareMap.get(Servo.class, "Turret-0");
    }
    public void Turret_ServoON(double set_speed){
        double current_speed;
        set_speed_drifter = set_speed;
        current_speed = Turret.getPosition() + set_speed;
        Turret.setPosition(current_speed);
    }

    public double get_angle(){
        return  90 - (180 * Turret.getPosition());
    }
    public void set_Position(double position){Turret.setPosition(position);}

    public double get_Position(){return Turret.getPosition();}
    public double get_power(){return set_speed_drifter;}

    public void to_position(double angle){
        double position = (90 - angle) / 180;
        Turret.setPosition(position);
    }
    public double get_exact_YAW(double theta){
        return  get_angle() + theta;
    }

    public double turret_free(double target , double old_position){
        if ((int)old_target != (int)target){
            int m = (int) Math.signum(target - old_position);
            old_target = target;
            return target + m * constant;
        }
        else {
            return target;
        }
    }

    public void stop(){
        Turret.setPosition(Turret.getPosition());
    }

}
