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
import org.firstinspires.ftc.teamcode.opmode.Calculate.Voltage_Drop;

@Config
public class PIDF_Shooter {
    public DcMotor shooter;
    public DcMotor shooter2;
    public double displacement, omega, current, previous_current, power, rpm, velocity, previous_time, delta_time, time_current, integral, derivative, error, previousError;
    ElapsedTime time;
    Follower follower;
    boolean critical = false;
    double omegaFiltered = 0;
    int index;

    // Encoder counts per revolution
    public static double PPR = 28;
    public static double alpha = 0.6;
    public static double kD = 0.000001;
    public static double kI = 0;
    public static double kP = 0.2;
    public static double kS = 0.043;
    public static double kV = 0.029;
    public static double radian = 48;
    public static double secondary_kD = 0.0001;
    public static double secondary_kI = 0;
    public static double secondary_kP = 0.7;
    public static double time_delay = 0.1;




    public double[] distance_list = {38.27,46.45,56.50,64.64,76.67,95.48,111.44,125.78,142.95,190.00,300.00};
    public double[] target_list =   {10.45,11.55,11.75,12.50,12.80,13.40,14.350,15.050,16.000,16.000,17.000};
    double[] voltage;
    double prev = 0.0;
    Distance distance = new Distance();
    Interpolation inter = new Interpolation();
    BinarySearch BS = new BinarySearch(distance_list);
    Voltage_Drop voltageDrop = new Voltage_Drop();



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
        voltageDrop.init_Voltage(hardwareMap);
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
            velocity = omega * (radian / 1000);
            previous_current = shooter.getCurrentPosition();
            previous_time = time_current;

        }

        if (velocity < 0.1){velocity = 0.0;}
        if (velocity == 0 && power > 0){
            velocity = prev;
        }
        else{
             prev = velocity;
        }
        return filter(velocity) ;// / 2.625935489 * 39.37
    }

    public double pidf(double targetVelocity) {
        double output = 0;
        error = Math.abs(targetVelocity) - velocity_info();

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

    public void run_shooter(double targetVelocity , boolean manual) {
        voltage = voltageDrop.Voltage_checker();
        power = pidf(targetVelocity);
        if (!is_working(power,velocity) || manual){
            power = manual(targetVelocity);
            critical = true;
        }
        else{critical = false;}

        if (voltage[1] < 11.5){power *= voltage[2];}

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

    //In dev
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
    public int get_index(){
        return index;
    }
    public double get_output(double targetVelocity) {
        return pidf(targetVelocity);
    }
    public double[] get_voltage(){return voltage;}
}
