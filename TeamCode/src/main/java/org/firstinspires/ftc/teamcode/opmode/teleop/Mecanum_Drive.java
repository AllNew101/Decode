package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class Mecanum_Drive extends OpMode {
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    private double A, B, C, D, XGame1, YGame1, XGame2, YGame2, SpeedMove, SpeedLimit, SpeedTurn, drs, dls;


    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
    }

    @Override
    public void start() {
    }


    @Override
    public void loop() {

    }

    public void move() {

        XGame1 = gamepad1.left_stick_x * -1;
        YGame1 = gamepad1.left_stick_y * -1;
        XGame2 = gamepad1.right_stick_x * -1;
        YGame2 = gamepad1.right_stick_y * -1;


            SpeedMove = Math.sqrt(Math.pow(XGame1, 2) + Math.pow(YGame1, 2)) * SpeedLimit;
            SpeedTurn = Math.sqrt(Math.pow(XGame2, 2) + Math.pow(YGame2, 2)) * (SpeedLimit - 0.1);
            drs = Math.sin((Math.atan2(YGame1, XGame1) / Math.PI * 180 - 45) / 180 * Math.PI);
            dls = Math.cos((Math.atan2(YGame1, XGame1) / Math.PI * 180 - 45) / 180 * Math.PI);
            if (YGame1 < 0) {
                if (dls >= drs) {
                    dls = -(dls / drs);
                    drs = -1;
                } else {
                    drs = -(drs / dls);
                    dls = -1;
                }
            } else {
                if (drs <= dls) {
                    drs = drs / dls;
                    dls = 1;
                } else {
                    dls = dls / drs;
                    drs = 1;
                }
            }

        A = Math.min(Math.max(drs * SpeedMove, -SpeedLimit), SpeedLimit);
        B = Math.min(Math.max(dls * SpeedMove, -SpeedLimit), SpeedLimit);
        C = Math.min(Math.max(dls * SpeedMove, -SpeedLimit), SpeedLimit);
        D = Math.min(Math.max(drs * SpeedMove, -SpeedLimit), SpeedLimit);
        A += Math.sin((Math.atan2(YGame2, XGame2) / Math.PI * 180 - 45) / 180 * Math.PI) * SpeedTurn;
        B += Math.cos((Math.atan2(YGame2, XGame2) / Math.PI * 180 - 45) / 180 * Math.PI) * SpeedTurn;
        C += Math.sin((Math.atan2(YGame2, XGame2) / Math.PI * 180 - 45) / 180 * Math.PI) * SpeedTurn;
        D += Math.cos((Math.atan2(YGame2, XGame2) / Math.PI * 180 - 45) / 180 * Math.PI) * SpeedTurn;


}}