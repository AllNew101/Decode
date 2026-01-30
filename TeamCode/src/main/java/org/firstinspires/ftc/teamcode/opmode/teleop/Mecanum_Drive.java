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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.opmode.Calculate.Distance;
import org.firstinspires.ftc.teamcode.opmode.Calculate.Dynamics;

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
    ElapsedTime time;

    //Tuning
    public static int key = 0;
    public static int position = 4;
    public static int speed_servo = 4;
    public static double target = 0.00;
    public static double speed_offset = 0.4;
    public static double speed_eshooter = 0.05;
    public static double[] multiplier = {1,1,1};
    public static boolean break_shooter = false;
    public static boolean is_red = true;
    public static int test = 0;


    private double offset = 0.0;
    double adj;
    double[] lead;
    boolean check_a = false;
    boolean check_B = false;
    boolean check_X = false;
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
        time.reset();

        Ying.init_vel(hardwareMap, follower, time);
        intake.init_intake(hardwareMap);
        angle.init_angular(hardwareMap);
        Turret.init_turret(hardwareMap, time);
        telemetryX.init(telemetry);
        closer.init_angular(hardwareMap);
        
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

//        //Automated PathFollowing
//        if (gamepad1.aWasPressed()) {
//            follower.followPath(pathChain.get());
//            automatedDrive = true;
//        }
//
//        //Stop automated following if the follower is done
//        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
//            follower.startTeleopDrive();
//            automatedDrive = false;
//        }

        //Slow Mode
        adj = Ying.distance_adjustment(follower.getPose().getX(), follower.getPose().getY(), is_red);
        lead = dynamics.lead_calculation(follower.getVelocity().getMagnitude(), follower.getVelocity().getTheta() * 180 / Math.PI, Ying.getVelocity() * (1 / 2.186) * 39.3700787, Turret.get_angle(), 45);
        if (gamepad1.rightBumperWasPressed()) {
            slowMode = !slowMode;
        }
        if (gamepad2.right_bumper) {
            target += speed_eshooter;
        }
        if (gamepad2.left_bumper) {
            target -= speed_eshooter;
        }

        if (gamepad1.crossWasPressed()) {
            check_a = !check_a;
        }
        if (check_a) {
            intake.intake(1);
        } else if (!check_a) {
            intake.stop_intake();
        }
        if (gamepad2.dpad_up) {
            angle.angular_on(speed_servo);
        } else if (gamepad2.dpad_down) {
            angle.angular_on(-speed_servo);
        }
        if (gamepad2.dpad_left) {
            offset -= speed_offset;
        } else if (gamepad2.dpad_right) {
            offset += speed_offset;
        }
        if (gamepad2.optionsWasPressed()){is_red = !is_red;}

        if (gamepad2.circleWasPressed()) {check_B = !check_B;}
        if (check_B) {Ying.run_shooter(adj + target);}//Ying.run_shooter(target);
        else if (!check_B) {Ying.stop_shooter(break_shooter);}

        if (gamepad2.squareWasPressed()) {check_X = !check_X;}
        if (check_X) {closer.open();}
        else if (!check_X) {closer.close();}

        if (gamepad2.triangleWasPressed()){check_turret = !check_turret;}
        if (check_turret){Turret.to_position(Turret.targeting(follower.getPose().getX(), follower.getPose().getY(), is_red, follower.getPose().getHeading() * 180 / Math.PI,offset));}
        else if (!check_turret){Turret.to_position(offset);}



        drawing.drawRobot(follower.getPose(),"red");
        drawing.sendPacket();

        //Call this once per loop
        telemetryX.update();
        follower.update();
        debug();
    }

    public void debug(){
        telemetryX.addData("lead",lead,2);
        telemetryX.addData("position",follower.getPose(),2);
        telemetryX.addData("position",follower.getPose(),2);
        telemetryX.addData("velocity",follower.getVelocity(),2);
        telemetryX.addData("automatedDrive",automatedDrive,2);
        telemetryX.addData("OFFSET",target,2);
        telemetryX.addData("target (m/s)",adj + target,2);
        telemetryX.addData("critical",Ying.get_critical(),2);
        telemetryX.addData("displacement",Ying.getDisplacement(follower.getPose().getX(),follower.getPose().getY()),2);

        if (is_red){
            telemetryX.addData("Alliance","Red",2);
            //gamepad0.RGB_SETUP(gamepad2,"red",10);
        }
        else {telemetryX.addData("Alliance","Blue",2);
            //gamepad0.RGB_SETUP(gamepad2,"blue",10);
        }
        switch (key){
            case 1:
                telemetryX.addData("velocity",Ying.getVelocity(),0);
                telemetryX.addData("current_position",Ying.getCurrentposition(),0);
                telemetryX.addData("current",Ying.get_output(target),0);
                telemetryX.addData("omega",Ying.getOmega(),0);
                break;
            case 2:
                telemetryX.addData("angle",Turret.get_angle(),0);
                telemetryX.addData("degree",Turret.get_degree(),0);
                telemetryX.addData("power",Turret.get_power(),0);
                break;
            case 3:
                telemetryX.addData("angular",angle.get_position(),0);
                break;


        }
    }
}