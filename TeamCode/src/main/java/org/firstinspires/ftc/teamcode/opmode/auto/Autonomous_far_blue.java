package org.firstinspires.ftc.teamcode.opmode.auto; // make sure this aligns with class location

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmode.Calculate.Distance;
import org.firstinspires.ftc.teamcode.opmode.system.PIDF_intake;
import org.firstinspires.ftc.teamcode.opmode.system.Closer;
import org.firstinspires.ftc.teamcode.opmode.system.Distance_Sensor;
import org.firstinspires.ftc.teamcode.opmode.system.PIDF_Shooter;
import org.firstinspires.ftc.teamcode.opmode.system.Turret;
import org.firstinspires.ftc.teamcode.opmode.system.angular_set;
import org.firstinspires.ftc.teamcode.opmode.system.localization_limelight;
import org.firstinspires.ftc.teamcode.opmode.system.telemetryX;
import org.firstinspires.ftc.teamcode.opmode.teleop.Mecanum_Drive;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;
/* git add .
 * git push origin master
 * git commit -m "***********" */

@Config
@Autonomous(name = "BLUE FAR")
public class Autonomous_far_blue extends OpMode {

    private DcMotor rightRear;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor leftFront;
    private Follower follower;
    private Supplier<PathChain> pathChain;
    private PIDF_Shooter Ying;
    private angular_set angle;
    private Turret Turret;
    private Closer closer;
    private telemetryX telemetryX;
    private Distance_Sensor distance_sensor;
    private Distance distance;
    private localization_limelight camera;
    private PIDF_intake intake_PID;
    private ElapsedTime time,delay,delay_loop;

    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState, pathMec, pathMec2;


    double maximum = 0.4;
    double minimum = 0.36;
    double tracking;
    double count = 0;
    boolean check_delay = false;
    boolean check_delay2 = false;
    private final Pose startPose = new Pose(8.000, -30.000, Math.toRadians(90));
    private final Pose Pre_keep1 = new Pose(34.000, -30.00, Math.toRadians(90));
    private final Pose keep1 = new Pose(36.000, -2.00, Math.toRadians(90));
    private final Pose shoot1 = new Pose(20.000, -34.000, Math.toRadians(90));
    private final Pose keep_loop = new Pose(8.000, 2.000, Math.toRadians(90)); //ชิดกำแพง
    private final Pose keep_loopsec = new Pose(28.000, 2.000, Math.toRadians(90)); //ไม่ชิดกำแพง
    private final Pose shoot_loop = new Pose(20.000, -34.000, Math.toRadians(90));
    ///////////////////////////////////////////////////////////////////////////////////
    private final Pose Final = new Pose(16,-18,Math.toRadians(90));
    //Bazier zone


    ////////////////////////////////////////////////////////////////////////////////////////
    private PathChain go_Pre_keep1,keeping1,Pre_loop,shooting1,keeping_loop,keeping_loopsec,shooting_loop,shooting_loopsec,finish;

    public void buildPaths() {
        go_Pre_keep1 = follower
                .pathBuilder()
                .addPath(new BezierLine(startPose, Pre_keep1))
                .setLinearHeadingInterpolation(startPose.getHeading(), Pre_keep1.getHeading())
                .build();
        keeping1 = follower
                .pathBuilder()
                .addPath(new BezierLine(Pre_keep1, keep1))
                .setLinearHeadingInterpolation(Pre_keep1.getHeading(), keep1.getHeading())
                .build();
        shooting1 = follower
                .pathBuilder()
                .addPath(new BezierLine(keep1, shoot1))
                .setLinearHeadingInterpolation(keep1.getHeading(), shoot1.getHeading())
                .build();
        Pre_loop = follower
                .pathBuilder()
                .addPath(new BezierLine(shoot1, keep_loop))
                .setLinearHeadingInterpolation(shoot1.getHeading(), keep_loop.getHeading())
                .build();
        /////////////////////////////////////////////////////////////////////////////////
        shooting_loop = follower
                .pathBuilder()
                .addPath(new BezierLine(keep_loop, shoot_loop))
                .setLinearHeadingInterpolation(keep_loop.getHeading(), shoot_loop.getHeading())
                .build();
        keeping_loopsec = follower
                .pathBuilder()
                .addPath(new BezierLine(shoot_loop, keep_loopsec))
                .setLinearHeadingInterpolation(shoot_loop.getHeading(), keep_loopsec.getHeading())
                .build();
        shooting_loopsec = follower
                .pathBuilder()
                .addPath(new BezierLine(keep_loopsec, shoot_loop))
                .setLinearHeadingInterpolation(keep_loopsec.getHeading(), shoot_loop.getHeading())
                .build();
        keeping_loop = follower
                .pathBuilder()
                .addPath(new BezierLine(shoot_loop, keep_loop))
                .setLinearHeadingInterpolation(shoot_loop.getHeading(), keep_loop.getHeading())
                .build();
        //////////////////////////////////////////////////////////////////////////////////
        finish = follower
                .pathBuilder()
                .addPath(new BezierLine(keep_loop, Final))
                .setLinearHeadingInterpolation(keep_loop.getHeading(), Final.getHeading())
                .build();
    }




    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: {
                setMecState(2);
                intake_PID.intake(1);
                closer.close();
                angle.setPosition(0.4);
                delay.reset();
                setPathState(201);
                break;
            }
            case 201 :{
                if (!follower.isBusy()){
                    if(delay.seconds() > 3){setPathState(101);}
                    break;}
            }
            case 101:
                if (!follower.isBusy()){
                    intake_PID.intake(0.78);
                    closer.open();
                    setMecintake_augularState(1);
                    delay.reset();
                    setPathState(2011);
                    break;
                }
            case 2011 :{
                if (!follower.isBusy()){if(delay.seconds() > 1){
                    closer.close();
                    setPathState(1);}
                    break;}}
            case 1:
                if (!follower.isBusy()){
                    angle.setPosition(0.4);
                    intake_PID.intake(1);
                    follower.setMaxPower(0.8);
                    follower.followPath(go_Pre_keep1);
                    setPathState(2);
                    break;
                }
            case 2:
                if (!follower.isBusy()){
                    follower.setMaxPower(0.9);
                    follower.followPath(keeping1);
                    setPathState(3);
                    break;}

            case 3:
                if (!follower.isBusy()){
                    intake_PID.intake(1);
                    angle.setPosition(0.4);
                    delay.reset();
                    follower.setMaxPower(0.9);
                    follower.followPath(shooting1);
                    setPathState(204);
                    break;}
            case 204 :{
                if (!follower.isBusy()){if(delay.seconds() > 1){setPathState(103);}
                    break;}}
            case 103:
                if (!follower.isBusy()){
                    intake_PID.intake(0.78);
                    closer.open();
                    setMecintake_augularState(1);
                    delay.reset();
                    setPathState(2044);
                    break;
                }
            case 2044 :{
                if (!follower.isBusy()){if(delay.milliseconds() > 800){
                    closer.close();
                    setPathState(20);}
                    break;}}
            case 20:
                if (!follower.isBusy()){
                    intake_PID.intake(1);
                    delay.reset();
                    angle.setPosition(0.4);
                    follower.setMaxPower(1);
                    follower.followPath(Pre_loop);
                    setPathState(200);
                    break;}
            case 200:
                if (!follower.isBusy()){
                    if (delay.seconds() > 1){
                        setPathState(4);}
                    break;}
                ////////////////////////////////////////////////////////////////////////////////////
            case 4:
                if (!follower.isBusy()){
                    intake_PID.intake(1);
                    delay.reset();
                    follower.setMaxPower(0.8);
                    follower.followPath(shooting_loop);
                    setPathState(205);
                    break;}
            case 205 :{
                if (!follower.isBusy()){if(delay.seconds() > 1){setPathState(104);}
                    break;}}
            case 104:
                if (!follower.isBusy()){
                    intake_PID.intake(0.78);
                    closer.open();
                    setMecintake_augularState(1);
                    delay.reset();
                    setPathState(2055);
                    break;
                }
            case 2055 :{
                if (!follower.isBusy()){if(delay.milliseconds() > 800){
                    closer.close();
                    setPathState(5);}
                    break;}}
            case 5:
                if (!follower.isBusy()){
                    count += 1;
                    if (count < 5){
                        delay.reset();
                        follower.setMaxPower(0.8);
                        follower.followPath(keeping_loopsec);
                        setPathState(206);}
                    else{
                        setPathState(10);
                    }
                    break;}
            case 206 :{
                if (!follower.isBusy()){if(delay.seconds() > 1){
                    closer.close();
                    setPathState(6);}
                    break;}}
            case 6:
                if (!follower.isBusy()){
                    intake_PID.intake(1);
                    delay.reset();
                    follower.setMaxPower(0.8);
                    follower.followPath(shooting_loop);
                    setPathState(207);
                    break;}
            case 207 :{
                if (!follower.isBusy()){if(delay.seconds() > 1){setPathState(105);}
                    break;}}
            case 105:
                if (!follower.isBusy()){
                    intake_PID.intake(0.78);
                    closer.open();
                    setMecintake_augularState(1);
                    delay.reset();
                    setPathState(2077);
                    break;
                }
            case 2077 :{
                if (!follower.isBusy()){if(delay.milliseconds() > 800){
                    closer.close();
                    setPathState(7);}
                    break;}}
            case 7:
                if (!follower.isBusy()){
                    count += 1;
                    if (count < 5){
                        delay.reset();
                        follower.setMaxPower(0.8);
                        follower.followPath(keeping_loop);
                        setPathState(208);}
                    else{
                        setPathState(10);
                    }
                    break;}
            case 208 :{
                if (!follower.isBusy()){if(delay.seconds() > 1){
                    closer.close();
                    setPathState(4);}
                    break;}}
            ///////////////////////////////////////////////////////////////////////////////////
            case 10:
                if (!follower.isBusy()){
                    follower.setMaxPower(0.8);
                    follower.followPath(finish,true);
                    setPathState(-1);
                    break;
                }
                //////////////////////////////////////////////////////////////////////
        }}
    public void mechanicPathUpdate(){
        switch (pathMec) {
            case 1:
                tracking = distance.targeting(follower.getPose().getX(), follower.getPose().getY(), false, follower.getPose().getHeading() / Math.PI * 180, -1, Turret.get_limit(),1);
                Ying.run_shooter(130, false, false);
                Turret.to_position(tracking, 0,1);
                break;
            case 2:
                tracking = distance.targeting(follower.getPose().getX(), follower.getPose().getY(), false, follower.getPose().getHeading() / Math.PI * 180, -13, Turret.get_limit(),1);
                Ying.run_shooter(130, false, false);
                Turret.to_position(tracking, 0,1);
                break;
        }
    }
    public void mechanicaugularPathUpdate(){
        switch (pathMec) {
            case 1:
                angle.angular_on(-1 * Mecanum_Drive.speed_servo, minimum, maximum );
                break;

        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/

    /**
     * These change the states of the paths and actions
     * It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    public void setMecState(int pMec) {
        pathMec = pMec;
    }
    public void setMecintake_augularState(int pMec2) {
        pathMec2 = pMec2;
    }


    @Override
    public void loop() {


        // These loop the movements of the robot
        follower.update();
        mechanicPathUpdate();
        mechanicaugularPathUpdate();
        autonomousPathUpdate();


        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetryX.addData("velocity_shooter",Ying.getVelocity_X(),2);
        telemetryX.update();
        telemetry.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        Master_variable master = new Master_variable();
        master.set_starting_point(4);

        pathTimer = new Timer();
        opmodeTimer = new Timer();


        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");

        distance = new Distance();
        Ying = new PIDF_Shooter();
        angle = new angular_set();
        Turret = new Turret();
        telemetryX = new telemetryX();
        closer = new Closer();
        time = new ElapsedTime();
        delay = new ElapsedTime();
        delay_loop =new ElapsedTime();
        distance_sensor = new Distance_Sensor();
        camera = new localization_limelight();
        intake_PID = new PIDF_intake();
        time.reset();
        delay.reset();
        delay_loop.reset();
        Ying.init_vel(hardwareMap, follower, time);
        intake_PID.init_PIDF_intake(hardwareMap, time);
        angle.init_angular(hardwareMap);

        Turret.init_turret(hardwareMap, time);
        closer.init_angular(hardwareMap);
        distance_sensor.init_Distance_senser(hardwareMap);
        camera.init(hardwareMap);
        telemetryX.init(telemetry);









    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
        setMecState(0);
    }


    @Override
    public void stop() {
    }

}
