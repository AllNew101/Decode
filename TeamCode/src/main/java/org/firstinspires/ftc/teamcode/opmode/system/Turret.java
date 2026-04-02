package org.firstinspires.ftc.teamcode.opmode.system;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.AnalogInput;
import org.firstinspires.ftc.teamcode.opmode.Calculate.Distance;



@Config
public class Turret {
    DcMotorEx turret;
    AnalogInput poten;
    private double integral, derivative, previousError, delta_time , time_current , previous_time;
    boolean critical;
    boolean checking = false;
    double critical_fx = 0.0;
    double previous = 0.0;
    int count = 0;
    ElapsedTime time;
    Distance distance = new Distance();



    public static double kS = 0.17;
    public static double kD = 1;
    public static double kD_secondary = 0.9;
    public static double kP = 0.02;
    public static double kP_secondary = 0.01;
    public static double limit = 120;
    public static double varikI_secondary = 0;
    public static double condition = 20;

    private double gear_motor = 39;
    private double gear_turret = 89;

    private double gear_potentiometer = 12;
    private double gear_potentiometer_out = 42;

    private double middle_poten = 1.107;

    private double Per_round = 537.7;
    private double power_turret = 0;
    private double kI_secondary = varikI_secondary;
    private double target_velo = 0;
    private double output = 0;
    private double offset = 0;
    public void init_turret(HardwareMap hardwareMap, ElapsedTime Time) {
        poten = hardwareMap.get(AnalogInput.class, "poten");
        turret = hardwareMap.get(DcMotorEx.class, "Turret");
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        turret.setDirection(DcMotorEx.Direction.REVERSE);
        offset = convert_potentiometer_to_degree(poten.getVoltage());

        time = Time;
    }

    public void to_position(double target){
        double error = target - (convert_current_to_degree(turret.getCurrentPosition()) + offset);
        if (Math.abs(error) < 1){
            error = 0;
        }
        double output = 0;
        time_current = time.seconds();
        delta_time = (time_current - previous_time) ;

        integral += error * delta_time;
        derivative = (error - previousError) / delta_time;


//        if (!is_working(output ,delta) || manual) {
//            count += 1;
//            if (count > 3) {critical = true;}
//            else{critical = false;}
//        }


        if (Math.abs(error) > condition) {
            turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            output = kP * error + kD * derivative + kS * Math.signum(error);
        }
        else {
            turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            output = kP_secondary * error + kI_secondary * integral + kD_secondary * derivative + kS * Math.signum(error);
        }



        turn(output);

        previousError = error;
        previous = turret.getCurrentPosition();
    }




    public void turn(double power){
        power_turret = power;
        boolean check_over = false;


        if (convert_current_to_degree(turret.getCurrentPosition()) > limit && power_turret > 0){check_over = true;}
        if (convert_current_to_degree(turret.getCurrentPosition()) < -limit && power_turret < 0){check_over = true;}
        if (convert_current_to_degree(turret.getCurrentPosition()) > limit && power_turret < 0){check_over = false;}
        if (convert_current_to_degree(turret.getCurrentPosition()) < -limit && power_turret > 0){check_over = false;}
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
    public double convert_potentiometer_to_degree(double position){
        return ((position - middle_poten) / poten.getMaxVoltage()) * 270 * gear_motor / gear_turret * gear_potentiometer_out / gear_potentiometer;
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
    public double get_poten_angle(){return  convert_potentiometer_to_degree(poten.getVoltage());}
    public double get_velocity() {return turret.getVelocity();}
    public double get_target_velocity() {return target_velo;}
    public double get_limit(){
        return limit;
    }
    public double get_output(){
        return output;
    }
    public double get_offset(){ return offset;}

}
