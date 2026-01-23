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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmode.system.PIDF_Shooter;
import org.firstinspires.ftc.teamcode.opmode.system.Turret;
import org.firstinspires.ftc.teamcode.opmode.system.Intake;
import org.firstinspires.ftc.teamcode.opmode.system.telemetryX;
import org.firstinspires.ftc.teamcode.opmode.system.angular_set;
import org.firstinspires.ftc.teamcode.opmode.system.Closer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@Config
@TeleOp
public class Mecanum_Drive extends OpMode {
    //Class import
    private Drawing drawing;
    private Follower follower;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private PIDF_Shooter Ying;
    private Intake intake;
    private angular_set angle;
    private Turret Turret;
    private Closer closer;
    private telemetryX telemetryX;

    //Tuning
    public static int key = 0;
    public static int position = 4;
    public static int speed_servo = 4;
    public static double target = 13.400000000000006;


    private double offset = 0.0;
    boolean check_a = false;
    boolean check_B = false;
    boolean check_X = false;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    public static Pose startingPose = new Pose(10, 10, Math.toRadians(0)); //See ExampleAuto to understand how to use this
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

        Ying.init_vel(hardwareMap);
        intake.init_intake(hardwareMap);
        angle.init_angular(hardwareMap);
        Turret.init_turret(hardwareMap);
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
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // Robot Centric
            );

                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
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
        if (gamepad1.rightBumperWasPressed()) {
            slowMode = !slowMode;
        }
        if (gamepad2.rightBumperWasPressed()) {
            target += 0.2;
        }
        if (gamepad2.leftBumperWasPressed()) {
            target -= 0.2;
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
            offset += 0.5;
        } else if (gamepad2.dpad_right) {
            offset -= 0.5;
        }
        Turret.to_position(offset );//(follower.getPose().getHeading() * 180 / Math.PI)




        if (gamepad2.circleWasPressed()) {check_B = !check_B;}
        if (check_B) {Ying.run_shooter(target);}
        else if (!check_B) {Ying.stop_shooter(false);}

        if (gamepad2.squareWasPressed()) {check_X = !check_X;}
        if (check_X) {closer.open();}
        else if (!check_X) {closer.close();}






        drawing.drawRobot(follower.getPose(),"red");
        drawing.sendPacket();

        telemetryX.update();
        //Call this once per loop
        follower.update();
        debug();

    }

    public void debug(){
        telemetryX.addData("position",follower.getPose(),2);
        telemetryX.addData("automatedDrive",automatedDrive,2);
        telemetryX.addData("target (m/s)",target,2);
        switch (key){
            case 1:
                telemetryX.addData("velocity",Ying.getVelocity(),2);
                telemetryX.addData("current_position",Ying.getCurrentposition(),2);
                telemetryX.addData("current",Ying.get_output(target),2);
                telemetryX.addData("omega",Ying.getOmega(),2);
                break;
            case 2:
                telemetryX.addData("angle",Turret.get_angle(),2);
                telemetryX.addData("degree",Turret.get_degree(),2);
                telemetryX.addData("power",Turret.get_power(),2);
                break;


        }
    }
}