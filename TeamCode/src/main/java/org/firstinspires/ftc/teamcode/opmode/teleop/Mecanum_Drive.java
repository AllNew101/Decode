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
import org.firstinspires.ftc.teamcode.opmode.system.localization_limelight;


import org.firstinspires.ftc.teamcode.opmode.Indev.Turret_servo;

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
    private Turret_servo Turret;
    private Closer closer;
    private Dynamics dynamics;
    private telemetryX telemetryX;
    private Distance_Sensor sensor;
    private Distance distance;
    private localization_limelight camera;
    ElapsedTime time;
    public static double[] multiplier = {1,1,1};

    //Tuning
    public static boolean break_shooter = false;
    public static boolean check_X = false;
    public static boolean check_intake = false;
    public static boolean check_loca = false;
    public static boolean check_out = false;
    public static boolean check_shooter = true;
    public static boolean is_red = true;
    public static int key = 1;
    public static double lock_position = 0.5799999999999998;
    public static boolean manual = false;
    public static double maximum = 0.9;
    public static double minimum = 0.7;
    public static double moving_Coff = 9;
    public static double origin = 0.5;
    public static int position = 4;
    public static double ratio_shooter = 0.8769999999999999;
    public static double reduce_coff = 0.8;
    public static double speed_eshooter = 0.003;
    public static double speed_intake_far = 0.6;
    public static double speed_intake_near = 0.8;
    public static double speed_offset = 0.4;
    public static double speed_servo = 16;
    public static boolean theseus = false;




    public double ratio = 1;



    boolean check_one = false;
    boolean can_reverse = true;
    private double offset = 0.0;
    int previous_key = 0;
    double adj, tracking;
    Pose lead, estimate;
    double[] esti;
    private boolean slowMode = false;
    private boolean check_turret = false;
    public static Pose startingPose = new Pose(109.01567398119123, -34.08150470219436 , Math.toRadians(0));
    private boolean automatedDrive = false;

    @Override
    public void init() {
        distance = new Distance();
        drawing = new Drawing();
        Ying = new PIDF_Shooter();
        intake = new Intake();
        angle = new angular_set();
        Turret = new Turret_servo();
        telemetryX = new telemetryX();
        closer = new Closer();
        time = new ElapsedTime();
        gamepad0 = new gamepad();
        dynamics = new Dynamics();
        sensor = new Distance_Sensor();
        camera = new localization_limelight();
        time.reset();

        check_intake = false;
        check_shooter = false;
        check_X = false;
        check_out = false;

        Ying.init_vel(hardwareMap, follower, time);
        intake.init_intake(hardwareMap);
        angle.init_angular(hardwareMap);

        //Turret.init_turret(hardwareMap, time);
        Turret.init_Turret_servo(hardwareMap);


        telemetryX.init(telemetry);
        closer.init_angular(hardwareMap);
        sensor.init_distance(hardwareMap);
        camera.init(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        Ying.start_shooter();
        Turret.set_Position(origin);
        angle.setOrigin();
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
        lead = dynamics.lead(follower.getPose(),follower.getVelocity(),Ying.getVelocity(),Turret.get_angle() / 180 * Math.PI, is_red);
        tracking = distance.targeting(follower.getPose().getX(), follower.getPose().getY(), is_red, follower.getPose().getHeading() / Math.PI * 180 , offset);
        adj = Ying.distance_adjustment(follower.getPose().getX(), follower.getPose().getY(), is_red);
        esti = camera.getRobotPoseFromCamera(follower.getPose().getHeading());

        if (gamepad2.right_bumper) {ratio_shooter += speed_eshooter;}
        if (gamepad2.left_bumper) {ratio_shooter -= speed_eshooter;}
        if (gamepad2.optionsWasPressed()){is_red = !is_red;}
        if (gamepad2.guideWasPressed()){manual = !manual;}
        if (gamepad2.circleWasPressed()) {check_shooter = !check_shooter;}
        if (gamepad1.crossWasPressed()) {check_intake = !check_intake;}
        if (gamepad1.circleWasPressed()) {check_out = !check_out;}

        if (check_intake && !check_X) {
            intake.intake(1);
            check_out = false;
        }
        else if (check_out) {
            intake.intake(-1);
            check_intake = false;
            check_one = false;
        }
        else if (!check_intake && !check_out) {
            intake.stop_intake();
        }


        if (gamepad2.dpad_left) {
            offset -= speed_offset;
        } else if (gamepad2.dpad_right) {
            offset += speed_offset;
        }


        if (gamepad2.dpad_up) {lock_position += 0.01;}
        else if (gamepad2.dpad_down) {lock_position -= 0.01;}



        if (check_shooter) {Ying.run_shooter(adj * ratio_shooter * ratio, angle.get_angle(), manual, can_reverse);}
        else if (!check_shooter) {Ying.stop_shooter(break_shooter);}

        if (gamepad2.squareWasPressed()) {
            check_X = !check_X;
            if (check_X == false){
                ratio = reduce_coff;
                can_reverse = false;
            }
            else{
                ratio = 1;
                can_reverse = true;
            }
        }
        if (check_X) {
            closer.open();
            ratio = 1;
            check_intake = true;
            if (follower.getPose().getX() > 40){
                if (theseus){
                angle.angular_on(-1 * Ying.getAcceleration() / 90 * moving_Coff, minimum ,maximum);}
                else {angle.angular_on(-1 * speed_servo, minimum, maximum);}
                intake.intake(speed_intake_near);
            }
            else{
                angle.setPosition(maximum);
                intake.intake(speed_intake_far);
            }
        }
        else if (!check_X) {
            closer.close();
            if (theseus) {angle.setPosition(lock_position);}
            else{angle.setOrigin();}
        }

        if (gamepad1.dpadDownWasPressed()) {
            check_loca = true;
            check_turret = false;
        }
        else {
            check_loca = false;
        }

        if (gamepad2.triangleWasPressed()){check_turret = !check_turret;}
        if (check_turret){
            ratio = 1;
            can_reverse = true;
            Turret.to_position(Turret.turret_free(tracking,Turret.get_angle()));

        }
        else if (!check_turret){
            if (esti[0] == 0.0 && check_loca){
                offset = 0;
            }
            Turret.to_position(offset);
            if (esti[0] == 0.0 && check_loca){
                estimate = new Pose(esti[1] ,esti[2], esti[3]);
                follower.setPose(estimate);
            }
        }

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
        telemetryX.addData("target (m/s)",adj * ratio_shooter,2);
        telemetryX.addData("displacement",Ying.getDisplacement(follower.getPose().getX(),follower.getPose().getY()),2);

        if (Ying.get_critical()){telemetryX.addData("Danger!!!!","Shooter is in manual mode",2);}
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
                telemetryX.addData("acceleration",Ying.getAcceleration(),0);
                telemetryX.addData("velocity_X",Ying.getVelocity_X(),0);
                telemetryX.addData("current_position",Ying.getCurrentposition(),0);
                telemetryX.addData("current",Ying.get_output(adj * ratio_shooter * ratio),0);
                telemetryX.addData("omega",Ying.getOmega(),0);
                telemetryX.addData("OFFSET",ratio_shooter,2);
                break;

            case 2:
                telemetryX.addData("angle",Turret.get_angle(),0);
                telemetryX.addData("degree",Turret.get_Position(),0);
                telemetryX.addData("power",Turret.get_power(),0);
                telemetryX.addData("Turret",tracking,0);
                break;

            case 3:
                telemetryX.addData("angular",angle.get_position(),0);
                telemetryX.addData("angle",angle.get_angle(),0);
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
                telemetryX.addData("DISTANCE_DOWN",sensor.get_distanceDown(),2);
                telemetryX.addData("DISTANCE_DOWN_is",sensor.Is_distanceDown(),2);
                telemetryX.addData("IS_three",sensor.Is_three(),2);
                break;
            case 6:
                telemetryX.addData("Bat",Ying.get_voltage()[0],2);
                telemetryX.addData("Voltage",Ying.get_voltage()[1],2);
                telemetryX.addData("Scale",Ying.get_voltage()[2],2);
                break;
            case 7:
                telemetryX.addData("available",esti[0],2);
                telemetryX.addData("X",esti[1],2);
                telemetryX.addData("Y",esti[2],2);
                telemetryX.addData("heading",esti[3],2);
                break;
        }
        previous_key = key;
    }
}