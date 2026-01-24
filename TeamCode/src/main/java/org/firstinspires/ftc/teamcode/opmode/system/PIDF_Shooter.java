package org.firstinspires.ftc.teamcode.opmode.system;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.opmode.Calculate.Distance;


@Config
public class PIDF_Shooter {
    public DcMotor shooter;
    public DcMotor shooter2;
    public double omega, current, previous_current, power, rpm, velocity, previous_time, delta_time, time_current, integral, derivative, error, previousError;
    ElapsedTime time;
    double omegaFiltered = 0;
    // Encoder counts per revolution
    public static double PPR = 28;
    public static double alpha = 0.6;
    public static double kD = 0.000001;
    public static double kI = 0;
    public static double kP = 0.5;
    public static double kS = 0.01;
    public static double kV = 0.037;
    public static double radian = 48;
    public static double secondary_kD = 0.0001;
    public static double secondary_kI = 0;
    public static double secondary_kP = 0.29;
    public static double time_delay = 0.1;



    Distance distance = new Distance();


    public void init_vel(HardwareMap hardwareMap){
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

        time = new ElapsedTime();
        time.reset();

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
            velocity = omega * (radian / 1000);
            previous_current = shooter.getCurrentPosition();
            previous_time = time_current;

        }
        return filter(velocity);
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
        shooter.setPower(power);
        shooter2.setPower(power);

    }

    public void setPower(double power) {
        shooter.setPower(power);
        shooter2.setPower(power);
    }

    //In dev
    public double distance_adjustment(double X, double Y, boolean is_red){
        double x = distance.distance(X,Y,is_red)[1];
        double y = 0.5*x;
        return y;
    }


    // get function
    public double getVelocity(){
        return velocity_info();
    }
    public int getCurrentposition(){
        return shooter.getCurrentPosition();
    }
    public double gettime(){
        return time_current;
    }
    public double getDelta_time(){
        return delta_time;
    }
    public double getOmega(){
        return omega;
    }
    public double get_output(double targetVelocity) {
        return pidf(targetVelocity);
    }
}
