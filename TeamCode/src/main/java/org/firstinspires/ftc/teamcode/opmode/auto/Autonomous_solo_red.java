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
import org.firstinspires.ftc.teamcode.opmode.Indev.PIDF_intake;
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
@Autonomous(name = "RED SOLO")
public class Autonomous_solo_red extends OpMode {

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


    double maximum = 0.18;
    double tracking;
    double count = 0;
    boolean check_delay = false;
    boolean check_delay2 = false;
    private final Pose startPose = new Pose(125.500, -132.000, Math.toRadians(-145));
    private final Pose scorepreload = new Pose(90.000, -100.000, Math.toRadians(-90));
    private final Pose keep1 = new Pose(85.000, -125.00, Math.toRadians(-90));
    private final Pose shoot1 = new Pose(90.000, -104.000, Math.toRadians(-90));
    private final Pose Pre_keep2 = new Pose(64.000, -100.000, Math.toRadians(-90));
    private final Pose keep2 = new Pose(64.000, -128.000, Math.toRadians(-90));
    private final Pose shoot2 = new Pose(90.000, -110.000, Math.toRadians(-90));
    private final Pose Pre_keep3 = new Pose(40.000, -100.000, Math.toRadians(-90));
    private final Pose keep3 = new Pose(40.000, -128.000, Math.toRadians(-90));
    private final Pose shoot3 = new Pose(90.000, -110.000, Math.toRadians(-90));
    /////////////////////////////////////////////////////////////////////////////////////
    private final Pose openhuman = new Pose(70.000, -130.500, Math.toRadians(-170));
    private final Pose keepopen = new Pose(14.000, -134.000, Math.toRadians(-180));
    private final Pose keepopensec = new Pose(12.000, -127.500, Math.toRadians(-180));
    ///////////////////////////////////////////////////////////1//////////////////////////
    private final Pose keeploop = new Pose(14.000, -142.000, Math.toRadians(-180));
    private final Pose keeploopsec = new Pose(12.000, -138.000, Math.toRadians(-180));
    private final Pose shootloop = new Pose(74.000, -95.000, Math.toRadians(-90));
    ///////////////////////////////////////////////////////////////////////////////////
    //Bazier zone

//    private final Pose keep3BE = new Pose(45.500,-130.000,Math.toRadians(-180));

    private final Pose keepopen_BE = new Pose(48.500,-150.000,Math.toRadians(-180));
    private final Pose keeploop_BE = new Pose(75.000,-138.500,Math.toRadians(-180));

    ////////////////////////////////////////////////////////////////////////////////////////
    private PathChain Path1,Path2,Path3,Path4,Path44,Path5,Path6,Path7,Path8,Path88,Path9,Path10,Path101,Path11,go_prekeep3,keeping3,shooting3;


    public void buildPaths() {
        Path1 = follower
                .pathBuilder()
                .addPath(new BezierLine(startPose, scorepreload))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorepreload.getHeading())
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(new BezierLine(scorepreload, keep1))
                .setLinearHeadingInterpolation(scorepreload.getHeading(), keep1.getHeading())
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(new BezierLine(keep1, shoot1))
                .setLinearHeadingInterpolation(keep1.getHeading(), shoot1.getHeading())
                .build();

        Path4 = follower
                .pathBuilder()
                .addPath(new BezierLine(shoot1, Pre_keep2))
                .setLinearHeadingInterpolation(shoot1.getHeading(), Pre_keep2.getHeading())
                .build();

        Path44 = follower
                .pathBuilder()
                .addPath(new BezierLine(Pre_keep2, keep2))
                .setLinearHeadingInterpolation(Pre_keep2.getHeading(), keep2.getHeading())
                .build();

        Path5 = follower
                .pathBuilder()
                .addPath(new BezierLine(keep2, shoot2))
                .setLinearHeadingInterpolation(keep2.getHeading(), shoot2.getHeading())
                .build();

        go_prekeep3 = follower
                .pathBuilder()
                .addPath(new BezierLine(shoot2, Pre_keep3))
                .setLinearHeadingInterpolation(shoot2.getHeading(), Pre_keep3.getHeading())
                .build();

        keeping3 = follower
                .pathBuilder()
                .addPath(new BezierLine(Pre_keep3, keep3))
                .setLinearHeadingInterpolation(Pre_keep3.getHeading(), keep3.getHeading())
                .build();

        shooting3 = follower
                .pathBuilder()
                .addPath(new BezierLine(keep3, shoot3))
                .setLinearHeadingInterpolation(keep3.getHeading(), shoot3.getHeading())
                .build();
////////////////////////////////////////////////////////////////////////////////////////

        Path6 = follower
                .pathBuilder()
                .addPath(new BezierLine(shoot3, openhuman))
                .setLinearHeadingInterpolation(shoot3.getHeading(), openhuman.getHeading())
                .build();

        Path7 = follower
                .pathBuilder()
                .addPath(new BezierCurve(openhuman,keepopen_BE,keepopen))
                .setLinearHeadingInterpolation(openhuman.getHeading(), keepopen.getHeading())
                .build();

        Path8 = follower
                .pathBuilder()
                .addPath(new BezierLine(keepopen,shootloop))
                .setLinearHeadingInterpolation(keepopen.getHeading(), shootloop.getHeading())
                .build();

        Path88 = follower
                .pathBuilder()
                .addPath(new BezierLine(keepopen,keepopensec))
                .setLinearHeadingInterpolation(keepopen.getHeading(), keepopensec.getHeading())
                .build();
/////////////////////////////////////////////////////////////////////////////////////////
        Path9 = follower
                .pathBuilder()
                .addPath(new BezierCurve(shootloop,keeploop_BE, keeploop))
                .setLinearHeadingInterpolation(shootloop.getHeading(), keeploop.getHeading())
                .build();

        Path10 = follower
                .pathBuilder()
                .addPath(new BezierLine(keeploop,shootloop))
                .setLinearHeadingInterpolation(keeploop.getHeading(), shootloop.getHeading())
                .build();

        Path101 = follower
                .pathBuilder()
                .addPath(new BezierLine(keeploop,keeploopsec))
                .setLinearHeadingInterpolation(keeploop.getHeading(), keeploopsec.getHeading())
                .build();
        /////////////////////////////////////////////////////////////////////////////////

    }



    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: {
                follower.setMaxPower(1);
                follower.followPath(Path1);
                setMecState(2);
                intake_PID.intake(1);
                closer.close();
                angle.setPosition(maximum);
                delay.reset();
                setPathState(101);
                break;
            }
            case 201 :{
                if (!follower.isBusy()){
                    if(delay.seconds() > 1){setPathState(101);}
                break;}
            }
            case 101:
                if (!follower.isBusy()){
                    intake_PID.intake(0.8);
                    closer.open();
                    setMecintake_augularState(1);
                    delay.reset();
                    setPathState(2011);
                    break;
                }
            case 2011 :{
                if (!follower.isBusy()){if(delay.seconds() > 1){
                    closer.close();
                    setPathState(4);}
                    break;}}
            case 1:
                if (!follower.isBusy()){
                    angle.setPosition(maximum);
                    closer.close();
                    intake_PID.intake(1);
                    follower.setMaxPower(1);
                    follower.followPath(Path2);
                    setPathState(2);
                    break;
                }
            case 2:
                if (!follower.isBusy()){
                    follower.setMaxPower(0.9);
                    follower.followPath(Path3);
                    delay.reset();
                    setPathState(202);
                    break;}

            case 202 :{
                if (!follower.isBusy()){if(delay.seconds() > 1){setPathState(102);}
                    break;}}
            case 102:
                if (!follower.isBusy()){
                    intake_PID.intake(0.8);
                    closer.open();
                    setMecintake_augularState(1);
                    delay.reset();
                    setPathState(2022);
                    break;
                }
            case 2022 :{
                if (!follower.isBusy()){if(delay.seconds() > 1){
                    closer.close();
                    setPathState(3);}
                    break;}}
            case 3:
                if (!follower.isBusy()){
                    intake_PID.intake(1);
                    angle.setPosition(maximum);
                    closer.close();
                    follower.setMaxPower(0.9);
                    follower.followPath(Path4,true);
                    setPathState(33);
                    break;}
            case 33:
                if (!follower.isBusy()){
                    intake_PID.intake(1);
                    follower.setMaxPower(0.7);
                    follower.followPath(Path44);
                    setPathState(203);
                    break;}
            case 203 :{
                if (!follower.isBusy()){if(delay.seconds() > 1.5){setPathState(4);}
                    break;}}
            case 4:
                if (!follower.isBusy()){
                    follower.setMaxPower(0.8);
                    follower.followPath(Path5);
                    delay.reset();
                    setPathState(204);
                    break;}
            case 204 :{
                if (!follower.isBusy()){if(delay.seconds() > 1){setPathState(103);}
                    break;}}
            case 103:
                if (!follower.isBusy()){
                    intake_PID.intake(0.8);
                    closer.open();
                    setMecintake_augularState(1);
                    delay.reset();
                    setPathState(2044);
                    break;
                }
            case 2044 :{
                if (!follower.isBusy()){if(delay.seconds() > 1){
                    closer.close();
                    setPathState(20);}
                    break;}}
            case 20:
                if (!follower.isBusy()){
                    intake_PID.intake(1);
                    angle.setPosition(maximum);
                    closer.close();
                    follower.setMaxPower(1);
                    follower.followPath(go_prekeep3,true);
                    setPathState(2020);
                    break;}
            case 2020:
                if (!follower.isBusy()){
                    intake_PID.intake(1);
                    follower.setMaxPower(0.8);
                    follower.followPath(keeping3);
                    setPathState(22);
                    break;}
            case 22:
                if (!follower.isBusy()){
                    follower.setMaxPower(0.8);
                    follower.followPath(shooting3);
                    delay.reset();
                    setPathState(220);
                    break;}
            case 220 :{
                if (!follower.isBusy()){if(delay.seconds() > 1){setPathState(120);}
                    break;}}
            case 120:
                if (!follower.isBusy()){
                    intake_PID.intake(0.8);
                    closer.open();
                    setMecintake_augularState(1);
                    delay.reset();
                    setPathState(2200);
                    break;
                }
            case 2200 :{
                if (!follower.isBusy()){if(delay.seconds() > 1){
                    closer.close();
                    setPathState(5);}
                    break;}}
            //////////////////////////////////////////////////////////////////////////
            case 5:
                if (!follower.isBusy()){
                    setMecState(1);
                    intake_PID.intake(1);
                    delay.reset();
                    follower.setMaxPower(0.8);
                    follower.followPath(Path6);
                    setPathState(205);
                    break;}
            case 205 :{
                if (!follower.isBusy()){if(delay.seconds() > 2){setPathState(6);}
                    break;}}
            case 6:
                if (!follower.isBusy()){
                    closer.close();
                    follower.setMaxPower(1);
                    follower.followPath(Path7);
                    setPathState(7);
                    break;}

            case 7:
                if (!follower.isBusy()){
                    if (distance_sensor.get_Front_dis() < 14 && distance_sensor.get_Center_dis() < 14) {
                        // go shooting
                        follower.setMaxPower(1);
                        follower.followPath(Path8);
                        setPathState(206);
                    }else{
                        // continue keep
                        delay_loop.reset();
                        check_delay = true;
//                        follower.setMaxPower(0.6);
//                        follower.followPath(Path88);
                        setPathState(77);
                    }
                    break;}

            case 77:
                if ((delay_loop.seconds() > 1.5 && check_delay ) || !follower.isBusy()){
                    check_delay = false;
                    follower.setMaxPower(1);
                    follower.followPath(Path8);
                    setPathState(206);
                    }
            case 206 :{
                if (!follower.isBusy()){if(delay.seconds() > 1){setPathState(104);}
                    break;}}
            case 104:
                if (!follower.isBusy()){
                    intake_PID.intake(0.8);
                    closer.open();
                    setMecintake_augularState(1);
                    delay.reset();
                    setPathState(2066);
                    break;
                }
            case 2066 :{
                if (!follower.isBusy()){if(delay.seconds() > 1){
                    closer.close();
                    setPathState(4);}
                    break;}}
            //////////////////////////////////////////////////////////////////////////

            case 8:
                if (!follower.isBusy()){
                    count += 1;
                    closer.close();
                    if (count < 2){
                        follower.setMaxPower(1);
                        follower.followPath(Path9);
                        setPathState(9);}
                    else {;}
                    break;}
            case 9:
                if (!follower.isBusy()){
                    if (distance_sensor.get_Front_dis() < 14 && distance_sensor.get_Center_dis() < 14) {
                        // go shooting
                        follower.setMaxPower(1);
                        follower.followPath(Path10);
                        setPathState(207);
                    }else{
                        // continue keep
                        delay.reset();
//                        follower.setMaxPower(0.6);
//                        follower.followPath(Path101);
                        setPathState(99);
                    }
                    break;}
            case 99:
                if (!follower.isBusy()){
                    if (delay.seconds() > 2){
                        delay_loop.reset();
                        check_delay2 = true;
                        setPathState(999);}
                }
            case 999:
                if ((delay_loop.seconds() > 1 && check_delay2 ) || !follower.isBusy()){
                        check_delay2 = false;
                        follower.setMaxPower(1);
                        follower.followPath(Path10);
                        setPathState(207);

                }
            case 207 :{
                if (!follower.isBusy()){if(delay.seconds() > 1){setPathState(105);}
                    break;}}
            case 105:
                if (!follower.isBusy()){
                    intake_PID.intake(0.8);
                    closer.open();
                    setMecintake_augularState(1);
                    delay.reset();
                    setPathState(2077);
                    break;
                }
            case 2077 :{
                if (!follower.isBusy()){if(delay.seconds() > 1){
                    closer.close();
                    setPathState(8);}
                    break;}}
            //////////////////////////////////////////////////////////////////////

    }}
    public void mechanicPathUpdate(){
        switch (pathMec) {
            case 1:
                tracking = distance.targeting(follower.getPose().getX(), follower.getPose().getY(), true, follower.getPose().getHeading() / Math.PI * 180, 1, Turret.get_limit(),1);
                Ying.run_shooter(108, false, false);
                Turret.to_position(tracking, 0,1);
                break;
            case 2:
                tracking = distance.targeting(follower.getPose().getX(), follower.getPose().getY(), true, follower.getPose().getHeading() / Math.PI * 180, 1, Turret.get_limit(),1);
                Ying.run_shooter(107.5, false, false);
                Turret.to_position(tracking, 0,1);
                break;
        }
    }
    public void mechanicaugularPathUpdate(){
        switch (pathMec) {
            case 1:
                angle.angular_on(-1 * Mecanum_Drive.speed_servo, 0.15, maximum );
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
        master.set_starting_point(1);

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
