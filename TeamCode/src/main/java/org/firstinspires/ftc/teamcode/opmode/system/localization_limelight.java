package org.firstinspires.ftc.teamcode.opmode.system;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
public class localization_limelight {
    private Limelight3A camera; //any camera here
    private Follower follower;
    private boolean following = false;
    private final Pose TARGET_LOCATION = new Pose(); //Put the target location here
    double X = 0.0;
    double Y = 0.0;
    double heading = 0.0;
    double ava = 1.0;
    double previous_fps = 0;
    double fps = 0;
    public static double offset_X = -5;
    public static double offset_Y = -5;


    public void init(HardwareMap hardwareMap) {
        camera = hardwareMap.get(Limelight3A.class, "limelight");
        camera.pipelineSwitch(0);
        camera.start();
    }



    public double[] getRobotPoseFromCamera(double angle) {
        LLResult result = camera.getLatestResult();
        LLStatus status = camera.getStatus();
        //fps = result.getCaptureLatency();
        if (result != null && result.isValid() ) {
            Pose3D botpose_mt = result.getBotpose();
            if (botpose_mt != null) {
                ava = 0.0;
                X = botpose_mt.getPosition().x;
                Y = botpose_mt.getPosition().y;
                heading = 360 - botpose_mt.getOrientation().getYaw(AngleUnit.DEGREES);
            }
        }
        else{
            ava = 1.0;
        }
        double[] n = {ava, 144 - (X * 39.37 + 72) + offset_X * Math.cos(Math.toRadians(angle)) - offset_Y * Math.sin(Math.toRadians(angle)), Y * -39.37 - 72 + offset_X * Math.sin(Math.toRadians(angle)) + offset_Y * Math.cos(Math.toRadians(angle)), Math.toRadians(-1 * heading)};

        //previous_fps = fps;
        return n;
    }

}
