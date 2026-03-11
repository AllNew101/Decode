package org.firstinspires.ftc.teamcode.opmode.system;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.opmode.Calculate.Distance;



@Config
public class Turret {
    DcMotor turret,current;
    private double integral, derivative, previousError, delta_time , time_current , previous_time;
    boolean critical;
    double critical_fx = 0.0;
    double previous = 0.0;
    int count = 0;
    ElapsedTime time;
    Distance distance = new Distance();

    public static double Per_round = 537.7;
    public static double condition = 10;
    public static double feedForward = 0;
    public static double gear_motor = 59;
    public static double gear_turret = 99;
    public static double kD = 1e-7;
    public static double kD_secondary = 9e-8;
    public static double kI = 0;
    public static double kP = 0.04;
    public static double kP_secondary = 0.045;
    public static double limit = 75;




    private double power_turret = 0;


    public void init_turret(HardwareMap hardwareMap, ElapsedTime Time) {
        turret = hardwareMap.get(DcMotor.class, "Turret");
        current = hardwareMap.get(DcMotor.class, "leftRear");
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);

        time = Time;
    }
    public void manual(double power){
        critical_fx = power;
    }

    public void to_position(double target, boolean manual){
        double delta = current.getCurrentPosition() - previous;
        double error = target - convert_current_to_degree(current.getCurrentPosition());
        double output = 0;
        time_current = time.seconds();
        delta_time = (time_current - previous_time) ;

        integral += error * delta_time;
        derivative = (error - previousError) / delta_time;


//        if (!is_working(output ,delta) || manual) {
//            count += 1;
//            if (count > 3) {
//                critical = true;
//                output = critical_fx;
//            }
//            else{
//                critical = false;
//            }
//        }
//        else{critical = false;
//        count = 0;
//        }


        if (Math.abs(error) > condition) {
            output = kP * error + kI * integral + kD * derivative + feedForward;
        }
        else {
            output = kP_secondary * error + kI * integral + kD_secondary * derivative + feedForward;
        }
        turn(output);
        previousError = error;
        previous = current.getCurrentPosition();
    }



    public void turn(double power){
        power_turret = power;
        boolean check_over = false;


        if (convert_current_to_degree(current.getCurrentPosition()) > limit && power_turret > 0){check_over = true;}
        if (convert_current_to_degree(current.getCurrentPosition()) < -limit && power_turret < 0){check_over = true;}
        if (convert_current_to_degree(current.getCurrentPosition()) > limit && power_turret < 0){check_over = false;}
        if (convert_current_to_degree(current.getCurrentPosition()) < -limit && power_turret > 0){check_over = false;}
        if (check_over){stop();}
        else{
            turret.setPower(power_turret);
        }
    }

    public boolean is_working(double power, double velocity){
        if (Math.abs(power) > 0.09 && Math.abs(velocity) == 0){return false;}
        else {return true;}
    }



    public void stop(){
        turret.setPower(0);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public double convert_current_to_degree(double position){
        return (position / Per_round) * 360 * gear_motor / gear_turret;
    }
    public double convert_d_2_c(double position){
        return position * 7.48;
    }
    public double get_degree(){return current.getCurrentPosition();}
    public double get_angle(){
        return convert_current_to_degree(current.getCurrentPosition());
    }
    public double get_power(){
        return power_turret;
    }
    public boolean get_critical(){
        return critical;
    }
    public double get_derivative(){
        return derivative;
    }
    public double get_limit(){
        return limit;
    }

    public void reset(){
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}
