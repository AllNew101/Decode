package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
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
import org.firstinspires.ftc.teamcode.opmode.system.telemetryX;
import org.firstinspires.ftc.teamcode.opmode.system.angular_set;
import org.firstinspires.ftc.teamcode.opmode.system.Closer;
import org.firstinspires.ftc.teamcode.opmode.system.gamepad;
import org.firstinspires.ftc.teamcode.opmode.system.Distance_Sensor;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.opmode.Calculate.Distance;
import org.firstinspires.ftc.teamcode.opmode.system.PIDF_intake;
import org.firstinspires.ftc.teamcode.opmode.system.localization_limelight;
import org.firstinspires.ftc.teamcode.opmode.system.Turret;
import org.firstinspires.ftc.teamcode.opmode.auto.Master_variable;
import org.firstinspires.ftc.teamcode.opmode.Calculate.Dynamics;
import java.util.function.Supplier;

@Config
@TeleOp
public class Mecanum_Drive extends OpMode {
    //Class import
    private Drawing drawing;
    private Follower follower;
    private Supplier<PathChain> Auto_drive;
    private PIDF_Shooter Ying;
    private angular_set angle;
    private Turret Turret;
    private Closer closer;
    private telemetryX telemetryX;
    private Distance_Sensor distance_sensor;
    private Distance distance;
    private localization_limelight camera;
    private PIDF_intake intake_PID;
    private gamepad gamepad0;
    ElapsedTime time;
    public static double[] multiplier = {1, 1, 0.5};

    //Tuning
    public static boolean debug = false;
    public static boolean is_red = true;
    public static int key = 1;
    public static boolean manual = false;
    public static double maximum = 0.34;
    public static double minimum = 0.3;
    public static double ratio_shooter = 1.1;
    public static double speed_eshooter = 0.003;
    public static double speed_servo = 10;
    public static double theta_autodrive = -54;
    public static double speed_offset = 0.8;
    public static double speed_intake_near = 1;
    public static double speed_intake_far = 0.75;
    public static double maximum_limit_servo = 0.45;
    public static double minimum_limit_servo = 0;


    private boolean check_X = false;
    private boolean check_intake = false;
    private boolean check_loca = false;
    private boolean check_out = false;
    private boolean check_shooter = false;
    private boolean is_trigger = false;
    private boolean is_trigger2 = false;

    double offset = 0;
    public int mode = 1;

    boolean check_one = false;
    boolean check_far = false;
    boolean check_reverse = false;
    boolean error = false;
    boolean error_sen = false;
    boolean can_reverse = true;
    boolean lock = true;
    int previous_key = 0;
    boolean is_inside = true;
    double adj, tracking;
    double x = 72;
    double y = -72;
    double theta = 0;

    double manual_turn = 0;
    Pose estimate, lead;
    double[] esti;
    public static Pose startingPose;
    private boolean check_turret = false;
    private boolean automatedDrive = false;

    @Override
    public void init() {
        ratio_shooter = 1;
        distance = new Distance();

        drawing = new Drawing();
        Ying = new PIDF_Shooter();
        angle = new angular_set();
        Turret = new Turret();
        telemetryX = new telemetryX();
        closer = new Closer();
        time = new ElapsedTime();
        distance_sensor = new Distance_Sensor();
        camera = new localization_limelight();
        intake_PID = new PIDF_intake();
        gamepad0 = new gamepad();

        time.reset();
        Master_variable master = new Master_variable();

        check_intake = false;
        check_shooter = false;
        check_X = false;
        check_out = false;

        Ying.init_vel(hardwareMap, follower, time);
        intake_PID.init_PIDF_intake(hardwareMap, time);
        angle.init_angular(hardwareMap);

        telemetryX.init(telemetry);
        closer.init_angular(hardwareMap);
        distance_sensor.init_Distance_senser(hardwareMap);
        camera.init(hardwareMap);

        follower = Constants.createFollower(hardwareMap);

        switch (master.getStarting_auto()){
            case 1:
                startingPose = new Pose(62.000, -98.000, Math.toRadians(-90));
                offset = 2;
                break;
            case 2:
                startingPose = new Pose(58.000, -28.000, Math.toRadians(90));
                offset = -2;
                break;
            case 3:
                startingPose = new Pose(2.000, -90.000, Math.toRadians(-90));
                offset = 0;
                break;
            case 4:
                startingPose = new Pose(13.000, -36.000, Math.toRadians(90));
                offset = 0;
                break;

            default:
                startingPose = new Pose(72, -72, Math.toRadians(0));
        }
        if (master.getStarting_auto() == 0){Turret.init_turret(hardwareMap, time);}
        else {Turret.init_turret_teleop(hardwareMap, time);}

        follower.setStartingPose(startingPose);
        follower.update();


    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        Ying.start_shooter();
        angle.setPosition(maximum);
        offset = 0;
    }

    @Override
    public void loop() {
        if (gamepad2.left_stick_y > 0.1){
            if (minimum > minimum_limit_servo){
            maximum -= 0.004;
            minimum -= 0.004;}
        }
        else if(gamepad2.left_stick_y < -0.1){
            if(maximum < maximum_limit_servo){
            maximum += 0.004;
            minimum += 0.004;}
        }
        if (!automatedDrive) {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * multiplier[0],
                    -gamepad1.left_stick_x * multiplier[1],
                    -gamepad1.right_stick_x * multiplier[2],
                    true // Robot Centric
            );
        }

        if (gamepad2.psWasPressed()){
            if (!error){
                error = true;
                error_sen = true;
                mode = 0;}
            else {
                error = false;
                error_sen = false;
                mode = 1;
            }
        }
        if (gamepad2.dpadDownWasPressed()){error_sen = !error_sen;}
        if (!error_sen){distance_sensor.check_led();}

        if (gamepad2.squareWasPressed()) {
            check_X = !check_X;
        }

        if (speed_intake_near > 1){
            speed_intake_near = 1;
        }
        if (speed_intake_far > 1){
            speed_intake_far = 1;
        }

        if (gamepad2.right_trigger > 0 && is_trigger2){
            speed_intake_near += 0.05;
            speed_intake_far += 0.05;
            is_trigger2 = false;}
        else if (gamepad2.left_trigger > 0 && is_trigger2){
            speed_intake_near -= 0.05;
            speed_intake_far -= 0.05;
            is_trigger2 = false;
        }else if (gamepad2.left_trigger + gamepad2.right_trigger <= 0){
            is_trigger2 = true;
        }
        if (gamepad1.right_trigger > 0 && is_trigger) {
            check_X = !check_X;
            is_trigger = false;
        } else if (gamepad1.right_trigger <= 0) {
            is_trigger = true;

            if (check_X) {
                closer.open();
                check_intake = true;
                gamepad2.rumble(100);

                if (follower.getPose().getX() > 40) {
                    angle.angular_on(-1 * speed_servo, minimum, maximum);
                    intake_PID.intake_near(speed_intake_near); // origin 0.8
                } else {
                    angle.angular_on(-1 * speed_servo, minimum, maximum);
                    intake_PID.intake_far(speed_intake_far); // origin0.7
                }
            } else if (!check_X) {
                closer.close();
                gamepad2.stopRumble();
                angle.setPosition(maximum);
                lock = true;
            }

        //lead = dynamics.lead(follower.getPose(),follower.getVelocity(),Ying.getVelocity(),Turret.get_angle() / 180 * Math.PI, is_red);
        tracking = distance.targeting(follower.getPose().getX(), follower.getPose().getY(), is_red, follower.getPose().getHeading() / Math.PI * 180, offset, Turret.get_limit(),mode);

        adj = Ying.distance_adjustment(follower.getPose().getX(), follower.getPose().getY(), is_red);
        esti = camera.getRobotPoseFromCamera(follower.getPose().getHeading());

        if (gamepad2.right_bumper) {
            ratio_shooter += speed_eshooter;
        }
        if (gamepad2.left_bumper) {
            ratio_shooter -= speed_eshooter;
        }
        if (gamepad2.optionsWasPressed()) {
            is_red = !is_red;
        }


        if (gamepad2.guideWasPressed()) {
            manual = !manual;
        }
        if (gamepad2.circleWasPressed()) {
            check_shooter = !check_shooter;
        }
        if (gamepad1.crossWasPressed()) {
            check_intake = !check_intake;
        }
        if (gamepad1.circleWasPressed()) {
            check_out = !check_out;
        }

        if (gamepad2.dpadUpWasPressed()) {
            check_far = !check_far;
            if (check_far){
                //// curve 4.7
                maximum = 0.45;
                minimum = 0.45;
                ratio_shooter = 1.17;// shoot speed far
                speed_intake_far = 0.8;

            }else{
                //// curve 5
                maximum = 0.34;
                minimum = 0.3;

                //// curve 4.7
//                maximum = 0.36;
//                minimum = 0.3;
                ratio_shooter = 1;// shoot speed near
                speed_intake_near = 0.85;

            }
        }



        if (check_intake && !check_X) {
            if (distance_sensor.led_status()){
                intake_PID.intake(0.7);
            }else{
                intake_PID.intake(1);
                check_out = false;}

        } else if (check_out) {
            intake_PID.intake(-1);
            check_intake = false;
            check_one = false;

        } else if (!check_intake && !check_out) {
            intake_PID.stop_intake();
        }


        if (gamepad2.dpad_left) {
            offset -= speed_offset;

        } else if (gamepad2.dpad_right) {
            offset += speed_offset;

        }


        if (check_shooter) {
            Ying.run_shooter(adj * ratio_shooter, manual, can_reverse);
        } else if (!check_shooter) {
            Ying.stop_shooter();
        }
        if (gamepad1.dpadDownWasPressed()) {
              check_loca = true;
        } else {
              check_loca = false;
        }

        if (check_loca) {
            estimate = new Pose(esti[1], esti[2], esti[3]);
            follower.setPose(estimate);
        }


        if (gamepad2.triangleWasPressed()) {
            check_turret = !check_turret;
        }
        if (gamepad2.crossWasPressed()){
            mode += 1;
            if (mode == 1){offset += 3;}
            else if (mode == 2) {offset -= 3;}
        }
        if(mode > 2){mode = 1;}

        if (gamepad1.squareWasPressed()){
            check_reverse = !check_reverse;
        }
        if(check_reverse){
            multiplier[0] = -1;
            multiplier[1] = -1;
        }
        else{
            multiplier[0] = 1;   
            multiplier[1] = 1;
        }

        if (!error) {
            if (check_turret) {
                Turret.to_position(tracking, Ying.getVelocity_X(), mode);
            } else {
                Turret.to_position(offset, Ying.getVelocity_X(), mode);
            }
        }else{
            if(gamepad2.dpad_right){
                Turret.turn(0.35,0);
            }
            else if (gamepad2.dpad_left){
                Turret.turn(-0.35,0);
            }
            else {Turret.stop();}
        }

            drawing.drawRobot(follower.getPose(), "red");
            drawing.sendPacket();

            //Call this once per loop
            debug(debug);
            standard();
            telemetryX.update();
            follower.update();


        }
    }
    public void standard(){
        if (!manual){telemetryX.addData("Mode","Automation Mode",2);}
        else{telemetryX.addData("Mode","Manual Mode",2);}


        telemetryX.addData("mode",mode,2);
        telemetryX.addData("check_cameraWorking" , esti[0],2);
        telemetryX.addData("velocity_X",Ying.getVelocity_X(),2);
        telemetryX.addData("target (m/s)",adj * ratio_shooter,2);
        telemetryX.addData("speed_shoot_near",speed_intake_near,2);
        telemetryX.addData("speed_shoot_far",speed_intake_far,2);
        telemetryX.addData("check_far",check_far,2);
        telemetryX.addData("Maximum",maximum,2);
        telemetryX.addData("ERROR_TURRET",error,2);

        if (Ying.get_critical()){telemetryX.addData("Danger!!!!","Shooter is in manual mode",2);}
        if (is_red){
            telemetryX.addData("Alliance","Red",2);
            gamepad0.RGB_SETUP(gamepad2,"red",2000);
        }
        else {telemetryX.addData("Alliance","Blue",2);
            gamepad0.RGB_SETUP(gamepad2,"blue",2000);
        }
    }
    public void debug(boolean debug){
        if (debug){
        if (previous_key != key){telemetryX.clear();}
        switch (key){
            case 1:
                telemetryX.addData("Turn",follower.getPose().getHeading(),2);
                telemetryX.addData("displacement",Ying.getDisplacement(follower.getPose().getX(),follower.getPose().getY()),2);
                telemetryX.addData("position",follower.getPose(),2);
                telemetryX.addData("velocity",Ying.getVelocity(),0);
                telemetryX.addData("current_position",Ying.getCurrentposition(),0);
                telemetryX.addData("current",Ying.get_output(adj * ratio_shooter),0);
                telemetryX.addData("omega",Ying.getOmega(),0);
                telemetryX.addData("OFFSET",ratio_shooter,2);
                break;

            case 2:

                telemetryX.addData("angle",Turret.get_angle(),0);
                telemetryX.addData("degree",Turret.get_degree(),0);
                telemetryX.addData("power",Turret.get_power(),0);
                telemetryX.addData("offset",offset,0);
                telemetryX.addData("Turret",tracking,0);
                telemetryX.addData("velocity_turret_target",Turret.get_target_velocity(),2);
                telemetryX.addData("output",Turret.get_output(),2);
                telemetryX.addData("offset_turret",Turret.get_offset(),2);
                break;

            case 3:
                telemetryX.addData("angular",angle.get_position(),0);
                telemetryX.addData("angle",angle.get_angle(),0);
                break;
            case 4:
                break;
            case 5:
                telemetryX.addData("Front_dissen",distance_sensor.get_Front_dis(),2);
                telemetryX.addData("Center_dissen",distance_sensor.get_Center_dis(),2);
                break;
            case 7:
                telemetryX.addData("available",esti[0],2);
                telemetryX.addData("X",esti[1],2);
                telemetryX.addData("Y",esti[2],2);
                telemetryX.addData("heading",esti[3],2);
                break;
            case 8:
                telemetryX.addData("velo",intake_PID.get_velocity(),2);
                telemetryX.addData("power",intake_PID.getpower(),2);

        }
        previous_key = key;
    }}
}