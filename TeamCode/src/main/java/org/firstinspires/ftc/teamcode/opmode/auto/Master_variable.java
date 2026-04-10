package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Master_variable {
    public static int starting_auto;
    public static double intake_shoot = 0.7;

    public void set_starting_point(int starting){
        starting_auto = starting;
    }
    public int getStarting_auto(){
        return starting_auto;
    }





}
