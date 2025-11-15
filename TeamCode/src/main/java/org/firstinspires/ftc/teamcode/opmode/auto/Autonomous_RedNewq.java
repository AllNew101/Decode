package org.firstinspires.ftc.teamcode.opmode.auto; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous(name = "AutoSample_All_RedNew")
    public class Autonomous_RedNewq extends OpMode {

        private DcMotor rightRear;
        private DcMotor rightFront;
        private DcMotor leftRear;
        private DcMotor leftFront;
        private Servo angle_1;
        private Servo araise;
        private Servo angle_2;
        private DcMotor front_motor;
        private DcMotor shooter1;
        private DcMotor center_motor;
        private DcMotor shooter2;

        private Follower follower;
        private Timer pathTimer, actionTimer, opmodeTimer;
        private int pathState;


        private final Pose startPose = new Pose(0, 0, Math.toRadians(0)); // Start Pose of our robot.
        private final Pose scorePose = new Pose(75, -4, Math.toRadians(-40)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
        private final Pose P_keep1 = new Pose(90, -15, Math.toRadians(-90));
        private final Pose keep1 = new Pose(85, -40, Math.toRadians(-90));
        private final Pose open_keep1 = new Pose(78, -50, Math.toRadians(-90));
        private final Pose scorePose2 = new Pose(80, -8, Math.toRadians(-46));
        private final Pose P_keep2 = new Pose(67.5, -15, Math.toRadians(-90));
        private final Pose keep2 = new Pose(67.5, -56, Math.toRadians(-90));
        private final Pose scorePose3 = new Pose(80, -8, Math.toRadians(-46));
        private final Pose P_keep3 = new Pose(46, -15, Math.toRadians(-88));
        private final Pose keep3 = new Pose(46, -56, Math.toRadians(-88));
        private final Pose scorePose4 = new Pose(-22, 22, Math.toRadians(-46));
        private final Pose Final = new Pose(-82, -16, Math.toRadians(-46));





    private Path scorePreload;
        private PathChain open,Pre_keep1, keep_1, scoring2, Pre_keep2, keep_2, scoring3,Pre_keep3,keep_3,scoring4,Finale;

        public void buildPaths() {
            /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
            scorePreload = new Path(new BezierLine(startPose, scorePose));
            scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

            Pre_keep1 = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose, P_keep1))
                    .setLinearHeadingInterpolation(scorePose.getHeading(),P_keep1.getHeading())
                    .build();
            keep_1 = follower.pathBuilder()
                    .addPath(new BezierLine(P_keep1, keep1))
                    .setLinearHeadingInterpolation(P_keep1.getHeading(),keep1.getHeading())
                    .build();
            open = follower.pathBuilder()
                    .addPath(new BezierLine(keep1, open_keep1))
                    .setLinearHeadingInterpolation(keep1.getHeading(),open_keep1.getHeading())
                    .build();
            scoring2 = follower.pathBuilder()
                    .addPath(new BezierLine(open_keep1, scorePose2))
                    .setLinearHeadingInterpolation(open_keep1.getHeading(),scorePose2.getHeading())
                    .build();
            Pre_keep2 = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose2, P_keep2))
                    .setLinearHeadingInterpolation(scorePose2.getHeading(),P_keep2.getHeading())
                    .build();
            keep_2 = follower.pathBuilder()
                    .addPath(new BezierLine(P_keep2, keep2))
                    .setLinearHeadingInterpolation(P_keep2.getHeading(),keep2.getHeading())
                    .build();
            scoring3 = follower.pathBuilder()
                    .addPath(new BezierLine(keep2, scorePose3))
                    .setLinearHeadingInterpolation(keep2.getHeading(),scorePose3.getHeading())
                    .build();
            Pre_keep3 = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose3, P_keep3))
                    .setLinearHeadingInterpolation(scorePose3.getHeading(),P_keep3.getHeading())
                    .build();
            keep_3 = follower.pathBuilder()
                    .addPath(new BezierLine(P_keep3, keep3))
                    .setLinearHeadingInterpolation(P_keep3.getHeading(),keep3.getHeading())
                    .build();
            scoring4 = follower.pathBuilder()
                    .addPath(new BezierLine(keep3, scorePose4))
                    .setLinearHeadingInterpolation(keep3.getHeading(),scorePose4.getHeading())
                    .build();
            Finale = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose4, Final))
                    .setLinearHeadingInterpolation(scorePose4.getHeading(),Final.getHeading())
                    .build();

    /*
    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */
            /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */

        }


        public void autonomousPathUpdate() throws InterruptedException {
            switch (pathState) {
                case 0:
                    front_motor.setPower(0.75);
                    center_motor.setPower(1);
                    shooter1.setPower(0.58);
                    shooter2.setPower(0.58);
                    araise.setPosition(0.48);
                    angle_1.setPosition(0.56);
                    angle_2.setPosition(0.56);
                    follower.setMaxPower(1);
                    follower.followPath(scorePreload);
                    setPathState(101);
                    break;


                case 101:
                    if (!follower.isBusy()){
                        follower.followPath(scorePreload);
                      Thread.sleep(250);

                      for (int i = 0;i < 3;i++){
                          switch (i) {
                              case 0:
                                  shooter1.setPower(0.55);
                                  shooter2.setPower(0.55);
                                  break;
                              case 1:
                                  shooter1.setPower(0.52);
                                  shooter2.setPower(0.52);
                                  break;
                              case 2:
                                  shooter1.setPower(0.52);
                                  shooter2.setPower(0.52);
                                  break;
                          }

                          araise.setPosition(0.94);
                          front_motor.setPower(0.7);
                          center_motor.setPower(0.6);
                          Thread.sleep(450);
                          araise.setPosition(0.48);
                          front_motor.setPower(1);
                          center_motor.setPower(1);
                          Thread.sleep(600);


                      }
                      setPathState(1);
                      break;
                    }
                case 1:
                    if (!follower.isBusy()){
                    front_motor.setPower(1);
                    follower.setMaxPower(1);
                    follower.followPath(Pre_keep1);
                    shooter1.setPower(0.52);
                    shooter2.setPower(0.52);
                    setPathState(2);
                    break;
                    }

                case 2:
                    if (!follower.isBusy()){
                       follower.setMaxPower(0.7);
                        follower.followPath(keep_1);
                        setPathState(21);
                        break;
                    }
                case 21:
                    if (!follower.isBusy()){
                        follower.setMaxPower(1);
                        follower.followPath(open);
                        setPathState(3);
                        break;
                    }
                case 3:
                    if (!follower.isBusy()){
                       Thread.sleep(200);
                        follower.setMaxPower(1);
                        follower.followPath(scoring2);
                        Thread.sleep(400);
                        setPathState(102);
                        break;
                    }
                case 102:
                    if (!follower.isBusy()){
                        follower.followPath(scorePreload);

                        for (int i = 0;i < 3;i++){
                            switch (i) {
                                case 0:
                                    shooter1.setPower(0.53);
                                    shooter2.setPower(0.53);
                                    break;
                                case 1:
                                    shooter1.setPower(0.53);
                                    shooter2.setPower(0.53);
                                    break;
                                case 2:
                                    shooter1.setPower(0.53);
                                    shooter2.setPower(0.53);
                                    break;
                            }

                            araise.setPosition(0.94);
                            front_motor.setPower(0.7);
                            center_motor.setPower(0.6);
                            Thread.sleep(450);
                            araise.setPosition(0.48);
                            front_motor.setPower(1);
                            center_motor.setPower(1);
                            Thread.sleep(600);


                        }
                        setPathState(4);
                        break;
                    }
                case 4:
                    if (!follower.isBusy()){
                        front_motor.setPower(1);
                        follower.setMaxPower(1);
                        follower.followPath(Pre_keep2);
                        shooter1.setPower(0.52);
                        shooter2.setPower(0.52);
                        setPathState(5);
                        break;
                    }

                case 5:
                    if (!follower.isBusy()){
//                    if ((follower.getPose().getX() < (P_keep2.getX() + 1) && (follower.getPose().getY()) < (P_keep2.getY()) + 1)){
                        follower.setMaxPower(0.5);
                        follower.followPath(keep_2);
                        setPathState(6);
                        break;
                    }
                case 6:
                    if (!follower.isBusy()){
//                    if ((follower.getPose().getX() < (keep2.getX() + 1) && (follower.getPose().getY()) < (keep2.getY()) + 1)){
                        Thread.sleep(200);
                        follower.setMaxPower(1);
                        follower.followPath(scoring3);

                        setPathState(103);
                        break;
                    }
                case 103:
                    if (!follower.isBusy()){
                        follower.followPath(scorePreload);

                        for (int i = 0;i < 3;i++){
                            switch (i) {
                                case 0:
                                    shooter1.setPower(0.52);
                                    shooter2.setPower(0.52);
                                    break;
                                case 1:
                                    shooter1.setPower(0.52);
                                    shooter2.setPower(0.52);
                                    break;
                                case 2:
                                    shooter1.setPower(0.52);
                                    shooter2.setPower(0.52);
                                    break;
                            }

                            araise.setPosition(0.94);
                            front_motor.setPower(0.7);
                            center_motor.setPower(0.6);
                            Thread.sleep(450);
                            araise.setPosition(0.48);
                            front_motor.setPower(1);
                            center_motor.setPower(1);
                            Thread.sleep(600);


                        }
                        setPathState(7);
                        break;
                    }

                case 7:
                    if (!follower.isBusy()){
                        follower.setMaxPower(1);
                        follower.followPath(Pre_keep3);
                        shooter1.setPower(0.5);
                        shooter2.setPower(0.5);
                        setPathState(8);
                        break;
                    }

                case 8:
                    if (!follower.isBusy()){

                        follower.setMaxPower(0.6);
                        follower.followPath(keep_3);
                        setPathState(9);
                        break;
                    }
                case 9:
                    if (!follower.isBusy()){

                        Thread.sleep(200);
                        follower.setMaxPower(1);
                        follower.followPath(scoring4);

                        setPathState(104);
                        break;
                    }
                case 104:
                    if (!follower.isBusy()){
                        follower.followPath(scorePreload);

                        for (int i = 0;i < 3;i++){
                            switch (i) {
                                case 0:
                                    shooter1.setPower(0.52);
                                    shooter2.setPower(0.52);
                                    break;
                                case 1:
                                    shooter1.setPower(0.52);
                                    shooter2.setPower(0.52);
                                    break;
                                case 2:
                                    shooter1.setPower(0.52);
                                    shooter2.setPower(0.52);
                                    break;
                            }

                            araise.setPosition(0.94);
                            front_motor.setPower(0.7);
                            center_motor.setPower(0.6);
                            Thread.sleep(450);
                            araise.setPosition(0.48);
                            front_motor.setPower(1);
                            center_motor.setPower(1);
                            Thread.sleep(600);


                        }
                        setPathState(-10);
                        break;
                    }
                case 10:
                    if (!follower.isBusy()){
                        follower.setMaxPower(1);
                        follower.followPath(Finale);

                        setPathState(11);
                        break;
                    }
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


        @Override
        public void loop() {


            // These loop the movements of the robot
            follower.update();

            try {
                autonomousPathUpdate();
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }

            // Feedback to Driver Hub
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
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
            angle_1 = hardwareMap.get(Servo.class, "angle_1");
            araise = hardwareMap.get(Servo.class, "araise");
            angle_2 = hardwareMap.get(Servo.class, "angle_2");
            front_motor = hardwareMap.get(DcMotor.class, "front_motor");
            shooter1 = hardwareMap.get(DcMotor.class, "shooter1");
            center_motor = hardwareMap.get(DcMotor.class, "center_motor");
            shooter2 = hardwareMap.get(DcMotor.class, "shooter2");

            shooter2.setDirection(DcMotor.Direction.REVERSE);
            angle_1.setDirection(Servo.Direction.REVERSE);
            araise.setDirection(Servo.Direction.REVERSE);










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

        }


        @Override
        public void stop() {
        }

    }
