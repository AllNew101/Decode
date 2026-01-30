package org.firstinspires.ftc.teamcode.opmode.system;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.opmode.Calculate.Distance;



@Config
public class Turret {
    DcMotor turret;
    private double integral, derivative, previousError, delta_time , time_current , previous_time;
    ElapsedTime time;
    Distance distance = new Distance();

    public static double feedForward = 0.001;
    public static double kD = 0;
    public static double kI = 0;
    public static double kP = 0.06;
    public static double limit = 105;


    private double power_turret = 0;


    public void init_turret(HardwareMap hardwareMap, ElapsedTime Time) {
        turret = hardwareMap.get(DcMotor.class, "turret1");
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        time = Time;
    }

    public void to_position(double target){
        double error = target - convert_c_2_d(turret.getCurrentPosition());
        time_current = time.seconds();
        delta_time = (time_current - previous_time) ;

        integral += error * delta_time;
        derivative = (error - previousError) / delta_time;
        double output = kP * error + kI * integral + kD * derivative + feedForward;

        turn(output);
        if (Math.abs(error) <= 0.05) {
            feedForward = 0;
        }
        previousError = error;
    }

    public double targeting(double X, double Y, boolean is_red , double theta, double offset){
        return 90 - distance.distance(X ,Y ,is_red)[2] + theta + offset;
    }



    public void turn(double power){
        power_turret = power;
        boolean check_over = false;
        if (Math.abs(convert_c_2_d(turret.getCurrentPosition())) > (limit * 0.95)){power_turret *= 0.05;}
        if (convert_c_2_d(turret.getCurrentPosition()) > limit && power > 0){check_over = true;}
        if (convert_c_2_d(turret.getCurrentPosition()) < -1 * limit && power < 0){check_over = true;}
        if (check_over){stop();}
        else{
            turret.setPower(power_turret);
            turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

    }




    public void stop(){
        turret.setPower(0);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public double convert_c_2_d(double position){
        return position / 7.48;
    }
    public double convert_d_2_c(double position){
        return position * 7.48;
    }
    public double get_degree(){
        return turret.getCurrentPosition();
    }
    public double get_angle(){
        return convert_c_2_d(turret.getCurrentPosition());
    }
    public double get_power(){
        return power_turret;
    }

    public void reset(){
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}
