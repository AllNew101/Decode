package org.firstinspires.ftc.teamcode.opmode.system;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.AnalogInput;
import org.firstinspires.ftc.teamcode.opmode.Calculate.Distance;



@Config
public class Turret {
    DcMotor turret;
    AnalogInput poten;
    private double  derivative, previousError, delta_time , time_current , previous_time;
    boolean critical;
    boolean checking = false;
    double critical_fx = 0.0;
    double previous = 0.0;
    ElapsedTime time;



    public static double condition = 15;
    public static double kD = 0.1;
    public static double kD_secondary = 0.01;
    public static double kP = 0.03;
    public static double kP_secondary = 0.035;
    public static double kS = 0.17;
    public static double kShooter = -0.000025;
    public static double limit = 179;
    public static double middle_poten = 1.365;



    private double gear_motor = 39;
    private double gear_turret = 89;

    private double gear_potentiometer = 12;
    private double gear_potentiometer_out = 42;

    public double limit_max = limit;
    public double limit_min = -limit;
    private double Per_round = 537.7;
    private double power_turret = 0;
    private double target_velo = 0;
    private double output = 0;
    private double offset = 0;
    public void init_turret(HardwareMap hardwareMap, ElapsedTime Time) {
        poten = hardwareMap.get(AnalogInput.class, "poten");
        turret = hardwareMap.get(DcMotor.class, "Turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);


        time = Time;
    }
    public void init_turret_teleop(HardwareMap hardwareMap, ElapsedTime Time) {
        poten = hardwareMap.get(AnalogInput.class, "poten");
        turret = hardwareMap.get(DcMotor.class, "Turret");

        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);

        time = Time;
    }

    public void to_position(double target, double velocity_shooter, int mode){
        if (mode == 1 || mode == 2){
            double error = target - (convert_current_to_degree(turret.getCurrentPosition()));
            if (Math.abs(error) < 1){error = 0;}
            double output = 0;

            time_current = time.seconds();
            delta_time = (time_current - previous_time) ;

            derivative = (error - previousError) / delta_time;

            if (mode == 1){if (Math.abs(error) > 170){error = 0;}}

            if (Math.abs(error) > condition) {
                output = kP * error + kD * derivative + kS * Math.signum(error) + kShooter * velocity_shooter * Math.signum(error);
            }
            else {
                output = kP_secondary * error + kD_secondary * derivative + kS * Math.signum(error) + kShooter * velocity_shooter * Math.signum(error);
                if (output > 0.3){power_turret = 0.3;}
                else if(output < -0.3){power_turret = -0.3;}
            }



            turn(output,mode);

            previousError = error;
            previous = turret.getCurrentPosition();}


    }




    public void turn(double power, int mode){
        power_turret = power;
        if (power_turret > 0.7){power_turret = 0.7;}
        else if(power_turret < -0.7){power_turret = -0.7;}

        boolean check_over = false;

        if (mode == 0) {
            check_over = false;
        }
        else if (mode == 1 || mode == 2){
            if (get_angle() > limit_max && power_turret >= 0) {check_over = true;}
            if (get_angle() < limit_min && power_turret <= 0) {check_over = true;}
            if (get_angle() > limit_max && power_turret < 0) {check_over = false;}
            if (get_angle() < limit_min && power_turret > 0) {check_over = false;}

            if (get_angle() > limit_max * 1.1) {power_turret = -0.25;}
            if (get_angle() < limit_min * 1.1) {power_turret = 0.25;}

            if (get_angle() > limit_max * 1.2) {check_over = true;}
            if (get_angle() < limit_min * 1.2) {check_over = true;}

        }
        else {check_over = true;}
        //////////////////////////////////////////////////////////

        if (check_over){stop();}
        else{
            turret.setPower(power_turret);}
    }


    public void stop(){
        turret.setPower(0);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public double convert_current_to_degree(double position){
        return (position / Per_round) * 360 * gear_motor / gear_turret;
    }

    public double get_degree(){return turret.getCurrentPosition();}
    public double get_angle(){
        return convert_current_to_degree(turret.getCurrentPosition()) + offset;
    }
    public double get_power(){
        return power_turret;
    }
    public boolean get_critical(){
        return critical;
    }
    public double get_target_velocity() {return target_velo;}
    public double get_limit(){
        return limit;
    }
    public double get_output(){
        return output;
    }
    public double get_offset(){ return offset;}

}
