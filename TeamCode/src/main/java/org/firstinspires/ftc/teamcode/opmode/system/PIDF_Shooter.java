package org.firstinspires.ftc.teamcode.opmode.system;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmode.Calculate.BinarySearch;
import org.firstinspires.ftc.teamcode.opmode.Calculate.Interpolation;
import org.firstinspires.ftc.teamcode.opmode.Calculate.Distance;

@Config
public class PIDF_Shooter {
    public DcMotor shooter;
    public DcMotor shooter2;
    public double  previous_velocity, omega, current, previous_current, power, rpm, velocity, previous_time, delta_time,delta_time_2,previous_time_2,current_time_2, current_time , integral, derivative, error, previousError;
    ElapsedTime time;
    Follower follower;
    boolean critical = false;
    int index;
    private double radian = 48;
    private double PPR = 28;

    // Encoder counts per revolution
    public static double defau = 58.6;
    public static double kD = 0.0001;
    public static double kI = 0;
    public static double kP = 0.04;
    public static double kS = 0.043;
    public static double kV = 0.00398;
    public static double secondary_kD = 0.00001;
    public static double secondary_kI = 0;
    public static double secondary_kP = 0.039;
    public static double time_delay = 0.08;



    public double[] distance_list = {0.0, 49.65, 58.0, 68.21, 82.8, 95.3, 190.00, 300.00};
    public double[] target_list =   {101.5, 101.5, 101.5, 102.2, 106.2, 106.2, 110.2, 110.2};
    double prev = 0.0;
    Distance distance = new Distance();
    Interpolation inter = new Interpolation();
    BinarySearch BS = new BinarySearch(distance_list);



    public void init_vel(HardwareMap hardwareMap, Follower position, ElapsedTime Time){
        // Hardware map change name here
        shooter2 = hardwareMap.get(DcMotor.class,"Shooter2");
        shooter = hardwareMap.get(DcMotor.class,"Shooter");

        // Direction set-up
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        time = Time;
        follower = position;
        //default position
        current = shooter.getCurrentPosition();
        previous_current = shooter.getCurrentPosition();
        previous_time = time.seconds();
        previous_velocity = 0;
    }

    public double velocity_info(){
        current = shooter.getCurrentPosition();
        current_time = time.seconds();
        delta_time = (current_time - previous_time) ;

        // V = wR Unit: m/s
        if (delta_time >= time_delay) {

            omega = ((((current - previous_current) / PPR) * (2 * Math.PI))) / delta_time;



            velocity = omega * (radian / 1000);
            previous_current = shooter.getCurrentPosition();
            previous_time = current_time;
        }

        if (velocity < 0.1){velocity = 0.0;}
        if (velocity == 0 && power > 0){
            velocity = prev;
        }
        else{
             prev = velocity;
        }
        return velocity * 0.381 * 39.37 ;
    }

    public double pidf(double targetVelocity, boolean can_reverse) {
        double output = 0;
        double velocity_shooter = velocity_info();
        error = Math.abs(targetVelocity) - (velocity_shooter * Math.cos(Math.toRadians(defau)));

        integral += error * delta_time;
        derivative = (error - previousError) / delta_time;

        if (Math.abs(velocity_info() * Math.cos(Math.toRadians(defau)) - targetVelocity) > 30) {output = kP * error + kI * integral + kD * derivative + (kV * targetVelocity + kS);}
        else {output = secondary_kP * error + secondary_kI * integral + secondary_kD * derivative + (kV * targetVelocity + kS);}

        previousError = error;
        return output;
    }

    public int start_shooter() {
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        integral = 0;
        derivative = 0;
        return 0;
    }

    public void stop_shooter() {
        shooter.setPower(0);
        shooter2.setPower(0);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    public void run_shooter(double targetVelocity, double theta, boolean manual, boolean can_reverse) {

        power = pidf(targetVelocity,can_reverse);
        if (!is_working(power,velocity) || manual){
            power = manual(targetVelocity);
            critical = true;
        }
        else{critical = false;}


        shooter.setPower(power);
        shooter2.setPower(power);
    }

    public void setPower(double power) {
        shooter.setPower(power);
        shooter2.setPower(power);
    }

    public boolean is_working(double power, double velocity){
        if (Math.abs(power) > kS && Math.floor(velocity) == 0){return false;}
        else {return true;}
    }

    public double manual(double targetVelocity){
        return  kV * targetVelocity + kS;
    }

    public double distance_adjustment(double X, double Y, boolean is_red){
        double displacement = distance.distance(X ,Y ,is_red)[3];
        if (displacement < distance_list[0]){displacement = distance_list[0];}
        if (displacement > distance_list[distance_list.length - 1]){displacement = distance_list[distance_list.length - 2];}

        index = BS.find(0,distance_list.length - 1,displacement);
        double target = inter.interpolation(displacement, distance_list[index], distance_list[index + 1], target_list[index], target_list[index + 1]) ;
        return target;
    }



    // get function
    public double getDisplacement(double x,double y){return distance.distance(x,y, true)[3];}
    public double getVelocity(){
        return velocity_info();
    }
    public double getVelocity_X(){
        return velocity_info() * Math.cos(Math.toRadians(defau));
    }
    public int getCurrentposition(){
        return shooter.getCurrentPosition();
    }
    public double get_time(){
        return current_time;
    }
    public double getDelta_time(){
        return delta_time;
    }
    public double getOmega(){
        return omega;
    }
    public boolean get_critical(){
        return critical;
    }
    public int get_index(){
        return index;
    }
    public double get_output(double targetVelocity) {
        return pidf(targetVelocity,true);
    }
}
