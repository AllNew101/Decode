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
@Autonomous(name = "Autonomous_opengate")
public class Autonomous_opengate extends OpMode {

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
    private ElapsedTime time,delay;

    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState, pathMec, pathMec2;

    double maximum = 0.18;
    double tracking;
    double count_loop = 0;
    private final Pose startPose = new Pose(125.500, -132.000, Math.toRadians(-145));
    private final Pose scorepreload = new Pose(90.000, -98.000, Math.toRadians(-90));
    private final Pose Pre_keep2 = new Pose(64.000, -100.000, Math.toRadians(-90));
    private final Pose keep2 = new Pose(63.000, -116.000, Math.toRadians(-90));
    private final Pose shoot2 = new Pose(90.000, -102.000, Math.toRadians(-90));
    /////////////////////////////////////////////////////////////////////////////////////
    private final Pose keepgate = new Pose(63.000, -134.000, Math.toRadians(-54));
    private final Pose shootloop = new Pose(90.000, -102.000, Math.toRadians(-90));
    /////////////////////////////////////////////////////////////////////////////////////
    private final Pose keeplast = new Pose(84.000, -125.00, Math.toRadians(-90));
    private final Pose shootlast = new Pose(90.000, -100.000, Math.toRadians(-90));


    //Bazier zone

//    private final Pose keep3BE = new Pose(45.500,-130.000,Math.toRadians(-180));
    private final Pose keepgate_BE = new Pose(40.000, -100.000, Math.toRadians(-55));



    ////////////////////////////////////////////////////////////////////////////////////////
    private PathChain Path1,Path2,Path22,Path3,Path4,Path44,Path5,Path6,Path7,Path8,Path9,Path10,Path11;


    public void buildPaths() {
        Path1 = follower
                .pathBuilder()
                .addPath(new BezierLine(startPose, scorepreload))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorepreload.getHeading())
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(new BezierLine(scorepreload, Pre_keep2))
                .setLinearHeadingInterpolation(scorepreload.getHeading(), Pre_keep2.getHeading())
                .build();

        Path22 = follower
                .pathBuilder()
                .addPath(new BezierLine(Pre_keep2, keep2))
                .setLinearHeadingInterpolation(Pre_keep2.getHeading(), keep2.getHeading())
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(new BezierLine(keep2, shoot2))
                .setLinearHeadingInterpolation(keep2.getHeading(), shoot2.getHeading())
                .build();
        Path4 = follower
                .pathBuilder()
                .addPath(new BezierCurve(shoot2, keepgate_BE,keepgate))
                .setLinearHeadingInterpolation(shoot2.getHeading(), keepgate.getHeading())
                .build();

////////////////////////////////////////////////////////////////////////////////////////

        Path5 = follower
                .pathBuilder()
                .addPath(new BezierLine(keepgate,shootloop))
                .setLinearHeadingInterpolation(keepgate.getHeading(), shootloop.getHeading())
                .build();

        Path6 = follower
                .pathBuilder()
                .addPath(new BezierCurve(shootloop,keepgate_BE,keepgate))
                .setLinearHeadingInterpolation(shootloop.getHeading(), keepgate.getHeading())
                .build();

//////////////////////////////////////////////////////////////////////////////////////////////

        Path7 = follower
                .pathBuilder()
                .addPath(new BezierLine(shootloop, keeplast))
                .setLinearHeadingInterpolation(shootloop.getHeading(), keeplast.getHeading())
                .build();

        Path8 = follower
                .pathBuilder()
                .addPath(new BezierLine(keeplast, shootlast))
                .setLinearHeadingInterpolation(keeplast.getHeading(), shootlast.getHeading())
                .build();
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
                setPathState(201);
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
                if (!follower.isBusy()){if(delay.seconds() > 1){setPathState(1);}
                    break;}}

            case 1:
                if (!follower.isBusy()){
                    intake_PID.intake(1);
                    angle.setPosition(maximum);
                    closer.close();
                    follower.setMaxPower(0.7);
                    follower.followPath(Path2,true);
                    setPathState(11);
                    break;}
            case 11:
                if (!follower.isBusy()){
                    intake_PID.intake(1);
                    follower.setMaxPower(0.8);
                    follower.followPath(Path22);
                    setPathState(2);
                    break;}
            case 2:
                if (!follower.isBusy()){
                    follower.setMaxPower(0.8);
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
                if (!follower.isBusy()){if(delay.seconds() > 1){setPathState(3);}
                    break;}}
            //////////////////////////////////////////////////////////////////////////
            case 3:
                if (!follower.isBusy()){
                    intake_PID.intake(1);
                    angle.setPosition(maximum);
                    closer.close();
                    delay.reset();
                    follower.setMaxPower(0.7);
                    follower.followPath(Path4,true);
                    setPathState(203);
                    break;}
            case 203 :{
                if (!follower.isBusy()){if(delay.seconds() > 2){setPathState(4);}
                    break;}}
            //////////////////////////////////////////////////////////////////////////
            case 4:
                if (!follower.isBusy()){
                    follower.setMaxPower(1);
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
                if (!follower.isBusy()){if(delay.seconds() > 1){setPathState(5);}
                    break;}}
            case 5:
                if (!follower.isBusy()){
                    intake_PID.intake(1);
                    angle.setPosition(maximum);
                    closer.close();
                    delay.reset();
//                    if (count_loop == 3){
//                        ;
//                    }else{
//                    count_loop += 1;
//                    follower.followPath(Path6);
//                    setPathState(4);}
                    follower.setMaxPower(0.7);
                    follower.followPath(Path6,true);
                    setPathState(205);
                    break;}
            case 205 :{
                if (!follower.isBusy()){if(delay.seconds() > 2){setPathState(4);}
                    break;}}
    }}
    public void mechanicPathUpdate(){
        switch (pathMec) {
            case 1:
                tracking = distance.targeting(follower.getPose().getX(), follower.getPose().getY(), true, follower.getPose().getHeading() / Math.PI * 180, -2, Turret.get_limit());
                Ying.run_shooter(116, false, false);
                Turret.to_position(tracking, 0);
                break;
            case 2:
                tracking = distance.targeting(follower.getPose().getX(), follower.getPose().getY(), true, follower.getPose().getHeading() / Math.PI * 180, -1, Turret.get_limit());
                Ying.run_shooter(105.5, false, false);
                Turret.to_position(tracking, 0);
                break;
        }
    }
    public void mechanicaugularPathUpdate(){
        switch (pathMec) {
            case 1:
                angle.angular_on(-1 * Mecanum_Drive.speed_servo, 0.15, maximum);
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
        distance_sensor = new Distance_Sensor();
        camera = new localization_limelight();
        intake_PID = new PIDF_intake();
        time.reset();
        delay.reset();
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
