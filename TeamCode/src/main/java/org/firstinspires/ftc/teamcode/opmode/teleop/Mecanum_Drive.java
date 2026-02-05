package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmode.system.PIDF_Shooter;
import org.firstinspires.ftc.teamcode.opmode.system.Turret;
import org.firstinspires.ftc.teamcode.opmode.system.Intake;
import org.firstinspires.ftc.teamcode.opmode.system.telemetryX;
import org.firstinspires.ftc.teamcode.opmode.system.angular_set;
import org.firstinspires.ftc.teamcode.opmode.system.Closer;
import org.firstinspires.ftc.teamcode.opmode.system.gamepad;
import org.firstinspires.ftc.teamcode.opmode.system.Distance_Sensor;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.opmode.Calculate.Distance;
import org.firstinspires.ftc.teamcode.opmode.Calculate.Dynamics;
import org.firstinspires.ftc.teamcode.opmode.Calculate.Voltage_Drop;

import java.util.function.Supplier;

@Config
@TeleOp
public class Mecanum_Drive extends OpMode {
    //Class import
    private Drawing drawing;
    private Follower follower;
    private Supplier<PathChain> pathChain;
    private gamepad gamepad0;
    private PIDF_Shooter Ying;
    private Intake intake;
    private angular_set angle;
    private Turret Turret;
    private Closer closer;
    private Dynamics dynamics;
    private telemetryX telemetryX;
    private Distance_Sensor sensor;
    ElapsedTime time;

    //Tuning
    public static int key = 0;
    public static int position = 4;
    public static double speed_servo = 16;
    public static double target = 0.00;
    public static double speed_offset = 0.4;
    public static double speed_eshooter = 0.05;
    public static double[] multiplier = {1,1,1};
    public static boolean break_shooter = false;
    public static boolean is_red = true;
    public static boolean manual = false;
    public static double speed_intake = 0.8;
    public static boolean check_intake = false;
    public static boolean check_shooter = false;
    public static boolean check_X = false;
    public static boolean check_out = false;

    boolean check_one = false;
    private double offset = 0.0;
    int previous_key = 0;
    double adj, tracking;
    Pose lead;
    private boolean slowMode = false;
    private boolean check_turret = false;
    public static Pose startingPose = new Pose(9.168, -79.602 , Math.toRadians(0)); //See ExampleAuto to understand how to use this
    private boolean automatedDrive = false;

    @Override
    public void init() {
        drawing = new Drawing();
        Ying = new PIDF_Shooter();
        intake = new Intake();
        angle = new angular_set();
        Turret = new Turret();
        telemetryX = new telemetryX();
        closer = new Closer();
        time = new ElapsedTime();
        gamepad0 = new gamepad();
        dynamics = new Dynamics();
        sensor = new Distance_Sensor();
        time.reset();

        check_intake = false;
        check_shooter = false;
        check_X = false;
        check_out = false;

        Ying.init_vel(hardwareMap, follower, time);
        intake.init_intake(hardwareMap);
        angle.init_angular(hardwareMap);
        Turret.init_turret(hardwareMap, time);
        telemetryX.init(telemetry);
        closer.init_angular(hardwareMap);
        sensor.init_distance(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        Ying.start_shooter();
    }

    @Override
    public void loop() {

        if (!automatedDrive) {
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * multiplier[0],
                    -gamepad1.left_stick_x * multiplier[1],
                    -gamepad1.right_stick_x * multiplier[2],
                    true // Robot Centric
            );

        }

        //Slow Mode
        lead = dynamics.lead(follower.getPose(),follower.getVelocity(),Ying.getVelocity() * (1 / 2.1486) * Math.cos(45) * 39.3701,Turret.get_angle() / 180 * Math.PI, is_red);
        tracking = Turret.targeting(follower.getPose().getX(), follower.getPose().getY(), is_red, follower.getPose().getHeading() / Math.PI * 180,offset);
        adj = Ying.distance_adjustment(follower.getPose().getX(), follower.getPose().getY(), is_red);

        if (gamepad2.right_bumper) {target += speed_eshooter;}
        if (gamepad2.left_bumper) {target -= speed_eshooter;}
        if (gamepad2.optionsWasPressed()){is_red = !is_red;}
        if (gamepad2.guideWasPressed()){manual = !manual;}
        if (gamepad2.circleWasPressed()) {check_shooter = !check_shooter;}
        if (gamepad1.crossWasPressed()) {check_intake = !check_intake;}
        if (gamepad1.circleWasPressed()) {check_out = !check_out;}

        if (check_intake) {
            intake.intake(speed_intake);}
        else if (check_out) {
            intake.intake(-1);
            check_one = false;
        }
        else if (!check_intake && !check_out) {
            intake.stop_intake();
        }
        if (sensor.Is_three() && !check_one){
            check_one = true;
        }




        if (gamepad2.dpad_left) {
            offset -= speed_offset;
            Turret.manual(-0.2);
        } else if (gamepad2.dpad_right) {
            offset += speed_offset;
            Turret.manual(0.2);
        }
        else {Turret.manual(0);}

        if (gamepad2.dpad_up) {speed_servo += 0.1;}
        else if (gamepad2.dpad_down) {speed_servo -= 0.1;}



        if (check_shooter) {Ying.run_shooter(adj + target, manual);}
        else if (!check_shooter) {Ying.stop_shooter(break_shooter);}
        //
        if (gamepad2.squareWasPressed()) {check_X = !check_X;}
        if (check_X) {
            closer.open();
            check_intake = true;
            if (sensor.check_time()){
                check_X = false;
                check_one = false;
                check_turret = false;
            }
        }
        else if (!check_X) {
            closer.close();
            angle.setOrigin();
        }
        if (follower.getPose().getX() > 40){
            angle.angular_on(-1 * speed_servo);
        }

        //
        if (gamepad2.triangleWasPressed()){check_turret = !check_turret;}
        if (check_turret){Turret.to_position(tracking,manual);}
        else if (!check_turret ){Turret.to_position(offset,manual);}

        drawing.drawRobot(follower.getPose(),"red");
        drawing.sendPacket();

        //Call this once per loop
        telemetryX.update();
        follower.update();
        debug();
    }

    public void debug(){
        if (previous_key != key){telemetryX.clear();}
        if (!manual){telemetryX.addData("Mode","Automation Mode",2);}
        else{telemetryX.addData("Mode","Manual Mode",2);}

        telemetryX.addData("position",follower.getPose(),2);
        telemetryX.addData("automatedDrive",automatedDrive,2);
        telemetryX.addData("target (m/s)",adj + target,2);
        telemetryX.addData("displacement",Ying.getDisplacement(follower.getPose().getX(),follower.getPose().getY()),2);

        if (Ying.get_critical()){telemetryX.addData("Danger!!!!","Shooter is in manual mode",2);}
        if (Turret.get_critical()){telemetryX.addData("Danger!!!!","Turret is in manual mode",2);}
        if (is_red){
            telemetryX.addData("Alliance","Red",2);
            gamepad0.RGB_SETUP(gamepad2,"red",2000);
        }
        else {telemetryX.addData("Alliance","Blue",2);
            gamepad0.RGB_SETUP(gamepad2,"blue",2000);
        }
        switch (key){
            case 1:
                telemetryX.addData("velocity",Ying.getVelocity(),0);
                telemetryX.addData("current_position",Ying.getCurrentposition(),0);
                telemetryX.addData("current",Ying.get_output(target),0);
                telemetryX.addData("omega",Ying.getOmega(),0);
                telemetryX.addData("OFFSET",target,2);
                break;

            case 2:
                telemetryX.addData("angle",Turret.get_angle(),0);
                telemetryX.addData("degree",Turret.get_degree(),0);
                telemetryX.addData("power",Turret.get_power(),0);
                break;

            case 3:
                telemetryX.addData("angular",angle.get_position(),0);
                break;

            case 4:
                telemetryX.addData("Velo",follower.getVelocity(),2);
                telemetryX.addData("Lead",lead.getPose(),2);
                telemetryX.addData("t",dynamics.get_t(),0);
                telemetryX.addData("a",dynamics.get_a(),0);
                telemetryX.addData("b",dynamics.get_b(),0);
                telemetryX.addData("c",dynamics.get_c(),0);
                telemetryX.addData("vb",dynamics.get_vb(),0);
                telemetryX.addData("dis",dynamics.get_dis(),0);
                telemetryX.addData("dot",dynamics.get_dot(follower.getVelocity()),0);
                break;

            case 5:
                telemetryX.addData("DISTANCE_UP",sensor.get_distanceUp(),2);
                telemetryX.addData("DISTANCE_DOWN",sensor.get_distanceDown(),2);
                telemetryX.addData("DISTANCE_UP_is",sensor.Is_distanceUp(),2);
                telemetryX.addData("DISTANCE_DOWN_is",sensor.Is_distanceDown(),2);
                telemetryX.addData("IS_three",sensor.Is_three(),2);
                break;
            case 6:
                telemetryX.addData("Bat",Ying.get_voltage()[0],2);
                telemetryX.addData("Voltage",Ying.get_voltage()[1],2);
                telemetryX.addData("Scale",Ying.get_voltage()[2],2);
                break;
        }
        previous_key = key;
    }
}