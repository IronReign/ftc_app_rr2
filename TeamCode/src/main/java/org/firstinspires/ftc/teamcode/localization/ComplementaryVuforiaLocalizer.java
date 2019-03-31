package org.firstinspires.ftc.teamcode.localization;

import com.acmerobotics.dashboard.config.Config;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.PoseBigWheel;
import org.firstinspires.ftc.teamcode.localization.Pose2d;
import org.firstinspires.ftc.teamcode.localization.Vector2d;
import org.firstinspires.ftc.teamcode.util.VuforiaFactory;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

@Config
public class ComplementaryVuforiaLocalizer {
    public static int CAMERA_FORWARD_DISPLACEMENT = 0; // displacement from center of robot in mm
    public static int CAMERA_VERTICAL_DISPLACEMENT = 0; // displacement from ground in mm
    public static int CAMERA_LEFT_DISPLACEMENT = 0;

    public static double LOW_FREQ_WEIGHT = 0.02;

    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;
    private static final float mmTargetHeight   = (6) * mmPerInch;

    private List<VuforiaTrackable> allTrackables;

    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;

    private Pose2d poseEstimate;

    private PoseBigWheel robot;
    private boolean visionLocalizeOnly = false;

    public static boolean DEBUG = false;
    private Pose2d highFreqPoseEstimate;
    private Pose2d lowFreqPoseEstimate;

    public ComplementaryVuforiaLocalizer(VuforiaFactory vuforia, boolean front, PoseBigWheel robot) {
        this.robot = robot;
        VuforiaLocalizer vuforiaLocalizer = VuforiaFactory.getVuforiaLocalizer();

        VuforiaTrackables targetsRoverRuckus = vuforiaLocalizer.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        allTrackables = new ArrayList<>();
        allTrackables.addAll(targetsRoverRuckus);

        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

        VuforiaLocalizer.CameraDirection cameraDirection = front ? FRONT : BACK;

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        cameraDirection == FRONT ? 90 : -90, 0, 0));


        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, cameraDirection);
        }

        targetsRoverRuckus.activate();
    }

    public Pose2d getPoseEstimate() { return poseEstimate; }

    public void enableVisionLocalizeOnly() { visionLocalizeOnly = true; } // we enable this when we get hit or something

    public void update() {
        robot.setEstimatedPose(poseEstimate);
        robot.updatePose();

        highFreqPoseEstimate = robot.getEstimatedPose();

        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                targetVisible = true;

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        Pose2d lowFreqPoseEstimate;
        if (targetVisible) {
            VectorF translation = lastLocation.getTranslation();
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);

            lowFreqPoseEstimate = new Pose2d(new Vector2d(translation.get(0) / mmPerInch, translation.get(1) / mmPerInch), rotation.thirdAngle);
            if (visionLocalizeOnly) {
                poseEstimate = lowFreqPoseEstimate;
                visionLocalizeOnly = false;
            } else {
                poseEstimate = lowFreqPoseEstimate.times(LOW_FREQ_WEIGHT).plus(highFreqPoseEstimate.times(1 - LOW_FREQ_WEIGHT));
            }
        } else {
            poseEstimate = highFreqPoseEstimate;
        }
    }

    public Pose2d getHighFreqPoseEstimate() {
        return highFreqPoseEstimate;
    }

    public Pose2d getLowFreqPoseEstimate() {
        return lowFreqPoseEstimate;
    }
}
