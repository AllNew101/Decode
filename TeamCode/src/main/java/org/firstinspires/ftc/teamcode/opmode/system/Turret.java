package org.firstinspires.ftc.teamcode.opmode.system;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Turret {
    DcMotor turret;
    public static double kp = 0.0;
    public static double kd = 0.0;
    public static double feedforward = 0.0;
    public static double limit = 110;

    public static double red_X =  0.0;
    public static double blue_X = 0.0;
    public static double red_Y = 0.0;
    public static double blue_Y = 0.0;

    private double X_target = 0.0;
    private double Y_target = 0.0;


    private TelemetryManager telemetryM;

    public void init_turret(HardwareMap hardwareMap) {
        turret = hardwareMap.get(DcMotor.class, "turret1");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void to_position(double target){
        double error = target - convert_c_2_d(turret.getCurrentPosition());

        turn(error * kp + feedforward);
        if (Math.abs(error) <= 0.05) {
            feedforward = 0;
        }
    }

    public double aimming(boolean red, double X, double Y, double theta, double offset){
        if (red){
            double X_target = red_X;
            double Y_target = red_Y;
        }
        else{
            double X_target = blue_X;
            double Y_target = blue_Y;
        }
        return 90 - Math.atan2(X_target - X, Y_target - Y)/ Math.PI * 180 + theta + offset;
    }

    public void turn(double power){
        turret.setPower(power);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (Math.abs(convert_c_2_d(turret.getCurrentPosition())) > limit){
            stop();
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
}
