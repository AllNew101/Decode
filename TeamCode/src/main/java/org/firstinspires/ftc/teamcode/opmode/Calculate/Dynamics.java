package org.firstinspires.ftc.teamcode.opmode.Calculate;

import org.firstinspires.ftc.teamcode.opmode.Calculate.Distance;

public class Dynamics {
    double theta = Distance.blue[2];
    public double[] velocity_calculation(double margin_robot, double theta_robot, double speed_shooter, double theta_shooter, double theta_field){
        double x_robot = margin_robot * Math.cos(Math.toRadians(theta_robot - theta_field));
        double y_robot = margin_robot * Math.sin(Math.toRadians(theta_robot - theta_field));
        double x_shooter = speed_shooter * Math.cos(Math.toRadians(theta_shooter - theta_field));
        double y_shooter = speed_shooter * Math.sin(Math.toRadians(theta_shooter - theta_field));
        double x_robot_final = x_robot + x_shooter;
        double y_robot_final = y_robot + y_shooter;

        double margin_final = Math.sqrt(Math.pow(x_robot_final,2) + Math.pow(y_robot_final,2));
        double theta_final = Math.atan2(y_robot_final , x_robot_final) * 180 / Math.PI ;
        double[] velocity = {x_robot_final, y_robot_final, margin_final, theta_final};
        return velocity;
    }
    public double[] lead_calculation(double margin_robot, double theta_robot, double speed_shooter, double theta_shooter, double theta_field){
        double[] velocity = velocity_calculation(margin_robot, theta_robot, speed_shooter, theta_shooter, theta_field);
        double Lead_turret = velocity[3] - theta_shooter;
        double Lead_shooter = velocity[1] * 1;
        double[] lead = {Lead_turret, Lead_shooter};
        return lead;
    }
}
