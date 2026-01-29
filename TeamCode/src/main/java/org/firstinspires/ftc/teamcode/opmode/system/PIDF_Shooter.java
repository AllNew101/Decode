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
    public double displacement, omega, current, previous_current, power, rpm, velocity, previous_time, delta_time, time_current, integral, derivative, error, previousError;
    ElapsedTime time;
    Follower follower;
    boolean critical = false;
    double omegaFiltered = 0;

    // Encoder counts per revolution
    public static double PPR = 28;
    public static double alpha = 0.6;
    public static double kD = 0.000001;
    public static double kI = 0;
    public static double kP = 0.5;
    public static double kS = 0.043;
    public static double kV = 0.037;
    public static double radian = 48;
    public static double secondary_kD = 0.0001;
    public static double secondary_kI = 0;
    public static double secondary_kP = 0.29;
    public static double time_delay = 0.1;

    public double[] distance_list = {20,40,60,80,100,120,140,160,180,200};
    public double[] target_list = {1,3,7,11,29,32,50,80,100,135};

    Distance distance = new Distance();
    Interpolation inter = new Interpolation();
    BinarySearch BS = new BinarySearch(distance_list);

    public void init_vel(HardwareMap hardwareMap, Follower position, ElapsedTime Time){
        // Hardware map change name here
        shooter2 = hardwareMap.get(DcMotor.class,"Shooter");
        shooter = hardwareMap.get(DcMotor.class,"Shooter2");

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
    }


    public double filter(double omegaRaw){
        omegaFiltered = alpha * omegaFiltered + (1 - alpha) * omegaRaw;
        return omegaFiltered;
    }

    public double velocity_info(){
        current = shooter.getCurrentPosition();
        time_current = time.seconds();
        delta_time = (time_current - previous_time) ;

        // V = wR Unit: m/s
        if (delta_time >= time_delay) {
            omega = ((((current - previous_current) / PPR) * (2 * Math.PI))) / delta_time;
            velocity = filter(omega * (radian / 1000));
            previous_current = shooter.getCurrentPosition();
            previous_time = time_current;

        }
        return velocity;
    }

    public double pidf(double targetVelocity) {
        double output = 0;
        error = targetVelocity - velocity_info();

        integral += error * delta_time;
        derivative = (error - previousError) / delta_time;

        if (Math.abs(velocity - targetVelocity) > 1) {output = kP * error + kI * integral + kD * derivative + (kV * targetVelocity + kS);}
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

    public void stop_shooter(boolean break_active) {
        shooter.setPower(0);
        shooter2.setPower(0);
        if (break_active){
            shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        else {
            shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

    }

    public void run_shooter(double targetVelocity) {
        double power = pidf(targetVelocity);
        if (!is_working(power,velocity)){
            power = critical(targetVelocity);
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

    public double critical(double targetVelocity){
        return  kV * targetVelocity + kS;
    }

    //In dev
    public double distance_adjustment(double X, double Y, boolean is_red){
        displacement = X;
        int index = BS.find(0,distance_list.length,displacement);
        double target = inter.interpolation(displacement,distance_list[index],distance_list[index + 1], target_list[index], target_list[index + 1]) ;
        return target;
    }


    // get function
    public double getDisplacement(double x,double y){return distance.distance(x,y, true)[3];}
    public double getVelocity(){
        return velocity_info();
    }
    public int getCurrentposition(){
        return shooter.getCurrentPosition();
    }
    public double get_time(){
        return time_current;
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
    public double get_output(double targetVelocity) {
        return pidf(targetVelocity);
    }
}
