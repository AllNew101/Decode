package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


public class PIDF_Shooter{
    private DcMotor shooter;
    private DcMotor shooter2;
    private double current, previous_current, power, rpm, velocity;
    private double PPR = 28;
    //Millimeters unit
    private double radian = 48;
    ElapsedTime time_arise;

    public void init_vel(HardwareMap Hardware){
        // Hardware map change name here
        shooter = Hardware.get(DcMotor.class,"shooter");
        shooter2 = Hardware.get(DcMotor.class,"shooter2");

        // Direction set-up
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        time_arise = new ElapsedTime();
        time_arise.reset();

        //default position
        current = shooter.getCurrentPosition();
        previous_current = shooter.getCurrentPosition();

    }
    public double change_velocity(){
        current = shooter.getCurrentPosition();
        //
        velocity = ((current - previous_current) / PPR) * (2 * Math.PI) * radian;
        previous_current = shooter.getCurrentPosition();
        return rpm;
    }
    public double pidf(){


        return 1.00;
    }
    public void runner(){
        shooter.setPower(pidf());
        shooter2.setPower(pidf());
    }
}
