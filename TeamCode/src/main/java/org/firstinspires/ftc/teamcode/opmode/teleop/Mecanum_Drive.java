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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@Config
@TeleOp
public class Mecanum_Drive extends OpMode {
    private Drawing drawing;
    private Follower follower;
    public static Pose startingPose = new Pose(10, 10, Math.toRadians(0)); //See ExampleAuto to understand how to use this
    private boolean automatedDrive = false;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    private PIDF_Shooter Ying;
    private Intake intake;
    public static double target = 10.00;

    boolean check_a = false;
    boolean check_B = false;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();


    @Override
    public void init() {
        drawing = new Drawing();
        Ying = new PIDF_Shooter();
        intake = new Intake();
        Ying.init_vel(hardwareMap);
        intake.init_intake(hardwareMap);



        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();


        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
        Ying.start_shooter();
    }

    @Override
    public void loop() {
        //Call this once per loop
         follower.update();

        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            //This is the normal version to use in the TeleOp
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

        //Automated PathFollowing
        if (gamepad1.aWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        //Slow Mode
        if (gamepad1.rightBumperWasPressed()) {
            slowMode = !slowMode;
        }

        if (gamepad1.aWasPressed()) {
            check_a = !check_a;
        }
        if (check_a) {
            intake.intake(1);
        } else if (!check_a) {
            intake.stop_intake();
        }
        if (gamepad1.dpad_up) {
            angle += 0.01;
        } else if (gamepad1.dpad_down) {
            angle += -0.01;
        }
        if (gamepad1.dpad_left) {
            turret.setPower(-0.4);
        } else if (gamepad1.dpad_right) {
            turret.setPower(0.4);
        } else {
            turret.setPower(0);
        }
        // Put loop blocks here.
        if (gamepad1.bWasPressed()) {
            check_B = !check_B;
        }
        if (check_B) {
            Ying.setPower(power);
        } else if (!check_B) {
            Ying.stop_shooter();
        }
        if (gamepad1.x) {
            close.setPosition(0.5);
        } else if (!gamepad1.x) {
            close.setPosition(0.5);
        }
        power += (gamepad1.right_trigger - gamepad1.left_trigger) * 0.005;
        telemetry.addData("power", power);
        telemetry.update();


        Ying.run_shooter(target);
        dashboardTelemetry.addData("target",target);
        dashboardTelemetry.addData("velo", Ying.filter(Ying.getVelocity()));
        dashboardTelemetry.addData("current",Ying.getCurrentposition());
        dashboardTelemetry.addData("current",Ying.get_output(target));
        dashboardTelemetry.addData("omega",Ying.getOmega());
        dashboardTelemetry.addData("position", follower.getPose());
        dashboardTelemetry.addData("automatedDrive", automatedDrive);
        drawing.drawRobot(follower.getPose(),"red");
        drawing.sendPacket();
        dashboardTelemetry.update();
    }
}