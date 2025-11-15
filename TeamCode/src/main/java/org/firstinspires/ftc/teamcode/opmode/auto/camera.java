package org.firstinspires.ftc.teamcode.opmode.auto;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Configurable
public class camera {
    AprilTagProcessor.Builder myAprilTagProcessorBuilder;
    VisionPortal.Builder myVisionPortalBuilder;
    boolean USE_WEBCAM;
    Position cameraPosition;
    ElapsedTime myElapsedTime;
    YawPitchRollAngles cameraOrientation;
    VisionPortal myVisionPortal;
    AprilTagDetection myAprilTagDetection;
    AprilTagProcessor myAprilTagProcessor;
    double right_power_camera;
    boolean control_orientation;
    double perfect_Y;
    double Error_ori;
    double KP_ori;
    double power_of_detec;
    double previous_error_ori;
    double angl = 0.0;
    double degree = 0.0;


    public void initAprilTag(HardwareMap hardwareMap) {


        USE_WEBCAM = true;
        cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
        cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);
        myElapsedTime = new ElapsedTime();
        // First, create an AprilTagProcessor.Builder.
        myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        myAprilTagProcessorBuilder.setCameraPose(cameraPosition, cameraOrientation);
        // Create an AprilTagProcessor by calling build.
        // Set the tag family to be detected.
        myAprilTagProcessorBuilder.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11);
        myAprilTagProcessor = myAprilTagProcessorBuilder.build();
        // Next, create a VisionPortal.Builder and set attributes related to the camera.
        myVisionPortalBuilder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            // Use a webcam.
            myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            // Use the device's back camera.
            myVisionPortalBuilder.setCamera(BuiltinCameraDirection.BACK);
        }
        // Add myAprilTagProcessor to the VisionPortal.Builder.
        myVisionPortalBuilder.addProcessor(myAprilTagProcessor);
        // Create a VisionPortal by calling build.
        myVisionPortal = myVisionPortalBuilder.build();
    }

    /**
     * Display info (using telemetry) for a recognized AprilTag.
     */
    private double ORIENTATION(double PERFECT) {
        double Error_ori;

        Error_ori = myAprilTagDetection.center.x - PERFECT;
        degree = Error_ori * 0.0859375;
        return degree;
    }



    public double telemetryAprilTag(double perfect) {
        List<AprilTagDetection> myAprilTagDetections;

        // Get a list of AprilTag detections.
        myAprilTagDetections = myAprilTagProcessor.getDetections();

        telemetry.addData("# AprilTags Detected", JavaUtil.listLength(myAprilTagDetections));

        // Iterate through list and call a function to display info for each recognized AprilTag.
        for (AprilTagDetection myAprilTagDetection_item : myAprilTagDetections) {
            myAprilTagDetection = myAprilTagDetection_item;
            // Display info about the detection.
            telemetry.addData("ID", myAprilTagDetection.id);
            if (myAprilTagDetection.id == 24 || myAprilTagDetection.id == 20) {
                return ORIENTATION(perfect);
            }
        }
        return 0;
    }



    /**
     * Describe this function...
     */
    public void distance_ap() {
        double dis_shoot;
        double theta_real_y;

        telemetry.addLine("X1:" + JavaUtil.formatNumber(myAprilTagDetection.corners[0].x, 6, 0) + "Y1:" + JavaUtil.formatNumber(myAprilTagDetection.corners[0].y, 6, 0) + "X2:" + JavaUtil.formatNumber(myAprilTagDetection.corners[1].x, 6, 0) + "Y2:" + JavaUtil.formatNumber(myAprilTagDetection.corners[1].y, 6, 0) + "X3:" + JavaUtil.formatNumber(myAprilTagDetection.corners[2].x, 6, 0) + "Y3:" + JavaUtil.formatNumber(myAprilTagDetection.corners[2].y, 6, 0) + "X4:" + JavaUtil.formatNumber(myAprilTagDetection.corners[3].x, 6, 0) + "Y4:" + JavaUtil.formatNumber(myAprilTagDetection.corners[3].y, 6, 0) + "-----------" + "   ");
        theta_real_y = 8.5 + ((480 - myAprilTagDetection.center.y) - 240) * 0.064583;
        telemetry.addLine("theta_real_y" + (myAprilTagDetection.center.y - 240) * 0.064583 + "theta_real_y" + theta_real_y + "tan:" + Math.tan(theta_real_y / 180 * Math.PI));
        dis_shoot = (0.746 - 0.31) / Math.tan(theta_real_y / 180 * Math.PI);
        telemetry.addLine("distance:" + dis_shoot);
    }

}