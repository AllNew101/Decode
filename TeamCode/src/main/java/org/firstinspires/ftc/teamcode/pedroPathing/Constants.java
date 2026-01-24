package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Constants {
    public static double forwardPodY = 2.25;
    public static double strafePodX = -7.75;

    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(-916.8404687)
            .lateralZeroPowerAcceleration(-79.8086021737)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.0001,0,0.001,0.0000002))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.2,0,0.01,0.028))
            .headingPIDFCoefficients(new PIDFCoefficients(1,0,0.0005,0.05))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1,0,0.08,0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.004,0,0.00013,0.6,0))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.00000002,0,0.0000001,0.6,0))
            .centripetalScaling(0.000084)
            .mass(11);


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear")
            .leftRearMotorName("leftRear")
            .leftFrontMotorName("leftFront")
            .xVelocity(41.743954)
            .yVelocity(55.14645)
            .useBrakeModeInTeleOp(true)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);


    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(forwardPodY)
            .strafePodX(strafePodX)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("odo")
            .yawScalar(1)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
