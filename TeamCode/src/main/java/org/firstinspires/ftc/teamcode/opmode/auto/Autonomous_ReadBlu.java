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


@Autonomous(name = "AutoSample_All_Blue")
    public class Autonomous_ReadBlu extends OpMode {

        private DcMotor rightRear;
        private DcMotor rightFront;
        private DcMotor leftRear;
        private DcMotor leftFront;
        private Servo angle_1;
        private Servo araise2;
        private Servo angle_2;
        private DcMotor front_motor;
        private DcMotor shooter1;
        private DcMotor center_motor;
        private DcMotor shooter2;

        private Follower follower;
        private Timer pathTimer, actionTimer, opmodeTimer;
        private int pathState;


        private final Pose startPose = new Pose(0, 0, Math.toRadians(-45)); // Start Pose of our robot.
        private final Pose scorePose = new Pose(-20, 20, Math.toRadians(-45)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
        private final Pose P_keep1 = new Pose(-36.8, 16, Math.toRadians(-89));
        private final Pose keep1 = new Pose(-39.8, -14, Math.toRadians(-89));
        private final Pose scorePose2 = new Pose(-20, 20, Math.toRadians(-48));
        private final Pose P_keep2 = new Pose(-59, 10, Math.toRadians(-87));
        private final Pose keep2 = new Pose(-59, -17, Math.toRadians(-87));
        private final Pose scorePose3 = new Pose(-22, 22, Math.toRadians(-48));
        private final Pose P_keep3 = new Pose(-82, 10, Math.toRadians(-88));
        private final Pose keep3 = new Pose(-82, -17, Math.toRadians(-88));
        private final Pose scorePose4 = new Pose(-22, 22, Math.toRadians(-46));
        private final Pose Final = new Pose(-82, -16, Math.toRadians(-46));





    private Path scorePreload;
        private PathChain Pre_keep1, keep_1, scoring2, Pre_keep2, keep_2, scoring3,Pre_keep3,keep_3,scoring4,Finale;

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
            scoring2 = follower.pathBuilder()
                    .addPath(new BezierLine(keep1, scorePose2))
                    .setLinearHeadingInterpolation(keep1.getHeading(),scorePose2.getHeading())
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
                    shooter1.setPower(0.55);
                    shooter2.setPower(0.55);
                    araise2.setPosition(0.25);
//                    angle_1.setPosition(0);
//                    angle_2.setPosition(0);
                    follower.setMaxPower(1);
                    follower.followPath(scorePreload);
                    setPathState(101);
                    break;


                case 101:
                    if (!follower.isBusy()){
                        follower.followPath(scorePreload);
                      Thread.sleep(500);

                      for (int i = 0;i < 3;i++){
                          switch (i) {
                              case 0:
                                  shooter1.setPower(0.55);
                                  shooter2.setPower(0.55);
                                  break;
                              case 1:
                                  shooter1.setPower(0.42);
                                  shooter2.setPower(0.42);
                                  break;
                              case 2:
                                  shooter1.setPower(0.42);
                                  shooter2.setPower(0.42);
                                  break;
                          }

                          araise2.setPosition(0.78);
                          front_motor.setPower(0.25);
                          center_motor.setPower(0.6);
                          Thread.sleep(460);
                          araise2.setPosition(0.48);
                          front_motor.setPower(0.75);
                          center_motor.setPower(1);
                          Thread.sleep(640);


                      }
                      setPathState(1);
                      break;
                    }
                case 1:
                    if (!follower.isBusy()){
                    follower.setMaxPower(1);
                    follower.followPath(Pre_keep1);
                    shooter1.setPower(0.45);
                    shooter2.setPower(0.45);
                    setPathState(2);
                    break;
                    }

                case 2:
                    if (!follower.isBusy()){
//                    if ((follower.getPose().getX() < (P_keep1.getX() + 1) && (follower.getPose().getY()) < (P_keep1.getY()) + 1)){
                        follower.setMaxPower(1);
                        follower.followPath(keep_1);
                        setPathState(3);
                        break;
                    }
                case 3:
                    if (!follower.isBusy()){
//                    if ((follower.getPose().getX() < (keep1.getX() + 1) && (follower.getPose().getY()) < (keep1.getY()) + 1)){
                        Thread.sleep(200);
                        follower.setMaxPower(1);
                        follower.followPath(scoring2);

                        setPathState(102);
                        break;
                    }
                case 102:
                    if (!follower.isBusy()){
//                    if ((follower.getPose().getX() > (scorePose2.getX() - 1) && (follower.getPose().getY()) > (scorePose2.getY()) - 1)) {
//                        Thread.sleep(300);
                        for (int i = 0;i < 3;i++){

                            araise2.setPosition(0.78);
                            front_motor.setPower(0.4);
                            center_motor.setPower(0.74);
                            Thread.sleep(340);
                            araise2.setPosition(0.48);
                            front_motor.setPower(0.75);
                            center_motor.setPower(1);
                            Thread.sleep(610);


                        }
                        setPathState(4);
                        break;
                    }
                case 4:
                    if (!follower.isBusy()){
                        follower.setMaxPower(1);
                        follower.followPath(Pre_keep2);
                        shooter1.setPower(0.45);
                        shooter2.setPower(0.45);
                        setPathState(5);
                        break;
                    }

                case 5:
                    if (!follower.isBusy()){
//                    if ((follower.getPose().getX() < (P_keep2.getX() + 1) && (follower.getPose().getY()) < (P_keep2.getY()) + 1)){
                        follower.setMaxPower(1);
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
//                    if ((follower.getPose().getX() > (scorePose3.getX() - 1) && (follower.getPose().getY()) > (scorePose3.getY()) - 1)) {
                        Thread.sleep(300);
                        for (int i = 0;i < 3;i++){

                            araise2.setPosition(0.78);
                            front_motor.setPower(0.4);
                            center_motor.setPower(0.74);
                            Thread.sleep(340);
                            araise2.setPosition(0.48);
                            front_motor.setPower(0.75);
                            center_motor.setPower(1);
                            Thread.sleep(610);


                        }
                        setPathState(7);
                        break;
                    }

                case 7:
                    if (!follower.isBusy()){
                        follower.setMaxPower(1);
                        follower.followPath(Pre_keep3);
                        shooter1.setPower(0.45);
                        shooter2.setPower(0.45);
                        setPathState(8);
                        break;
                    }

                case 8:
                    if (!follower.isBusy()){
//                    if ((follower.getPose().getX() < (P_keep3.getX() + 1) && (follower.getPose().getY()) < (P_keep3.getY()) + 1)){
                        follower.setMaxPower(1);
                        follower.followPath(keep_3);
                        setPathState(9);
                        break;
                    }
                case 9:
                    if (!follower.isBusy()){
//                    if ((follower.getPose().getX() < (keep3.getX() + 1) && (follower.getPose().getY()) < (keep3.getY()) + 1)){
                        Thread.sleep(200);
                        follower.setMaxPower(1);
                        follower.followPath(scoring4);

                        setPathState(104);
                        break;
                    }
                case 104:
                    if (!follower.isBusy()){
//                    if ((follower.getPose().getX() > (scorePose4.getX() - 1) && (follower.getPose().getY()) > (scorePose4.getY()) - 1)){
//                        Thread.sleep(300);
                        for (int i = 0;i < 3;i++){


                            araise2.setPosition(0.78);
                            front_motor.setPower(0.4);
                            center_motor.setPower(0.74);
                            Thread.sleep(340);
                            araise2.setPosition(0.48);
                            front_motor.setPower(0.75);
                            center_motor.setPower(1);
                            Thread.sleep(610);


                        }
                        setPathState(10);
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
            araise2 = hardwareMap.get(Servo.class, "araise2");
            angle_2 = hardwareMap.get(Servo.class, "angle_2");
            front_motor = hardwareMap.get(DcMotor.class, "front_motor");
            shooter1 = hardwareMap.get(DcMotor.class, "shooter1");
            center_motor = hardwareMap.get(DcMotor.class, "center_motor");
            shooter2 = hardwareMap.get(DcMotor.class, "shooter2");

            shooter2.setDirection(DcMotor.Direction.REVERSE);
            angle_1.setDirection(Servo.Direction.REVERSE);
            araise2.setDirection(Servo.Direction.REVERSE);










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
