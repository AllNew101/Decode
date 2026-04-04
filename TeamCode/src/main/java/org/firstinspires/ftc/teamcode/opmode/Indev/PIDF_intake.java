package org.firstinspires.ftc.teamcode.opmode.Indev;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class PIDF_intake {
    private double error;
    private double delta_error;
    private double previous_error;
    private double current ;
    private double previous ;
    private double delta_position;
    private double current_time ;
    private double previous_time ;
    private double delta_time;
    private double time_delay = 0.05;
    private double output = 0.0;


    public static double D = 0.001;
    public static double F = 0.7;
    public static double I = 0;
    public static double KS = 0.18;
    public static double P = 0.07;

    public static double near = 0.8;
    public static double far = 0.55;

    ElapsedTime time;
    DcMotor Front;

    public void init_PIDF_intake (HardwareMap hardwareMap, ElapsedTime Time){
        Front = hardwareMap.get(DcMotor.class, "Front");
        Front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        time = Time;
        previous_time = time.seconds();
        previous = Front.getCurrentPosition();

    }

    public double get_velocity (){
        current = Front.getCurrentPosition();
        current_time = time.seconds();
        delta_time = current_time - previous_time ;
        if (delta_time >= time_delay){
            delta_position = current - previous;
            previous = Front.getCurrentPosition();
        }
        return (delta_position / 145.1) / current_time;
    }

    public double PIDF (double target){
        error = target - get_velocity();
        delta_error = error - previous_error;
        output = P*error + D*delta_error/delta_time + (F * target + KS);
        previous_error = error;
        return output ;

    }

    public void start_intakepidf(double target){
        Front.setPower(PIDF(target));
        Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void intake_near(){
        Front.setPower(PIDF(near));
        Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void intake_far(){
        Front.setPower(PIDF(far));
        Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void intake(double target){
        Front.setPower(target);
        Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void stop_intake(){
        Front.setPower(0);
        Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public double getvelo() {return get_velocity();}
    public double getpower() {return output;}

}
