package org.firstinspires.ftc.teamcode.opmode.teleop;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.draw;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawOnlyCurrent;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.telemetryM;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.lang.reflect.Array;
import java.util.Arrays;
import java.util.function.Supplier;

@Configurable
@Disabled
@TeleOp
public abstract class Mecanum_Drive extends OpMode {
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    public double SpeedLimit = 1;
    public double A, B, C, D, XGame1, YGame1, XGame2, YGame2, SpeedMove, SpeedTurn, drs, dls;

    @Override
    public void init_loop(){
        telemetryM.debug("This will print your robot's position to telemetry while "
                + "allowing robot control through a basic mecanum drive on gamepad 1.");
        telemetryM.update(telemetry);
        follower.update();
        drawOnlyCurrent();
    }

    public double[] move() {
        this.SpeedLimit = SpeedLimit;
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

        draw();
        return new double[] {A,B,C,D};
}}