package org.firstinspires.ftc.teamcode;

/**
 * Created by karim on 2/10/2017.
 */

import android.support.annotation.Nullable;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import org.firstinspires.ftc.teamcode.util.VortexUtils;

/**
 * Created by FIXIT on 16-10-07.
 */
//@Autonomous
//public class BeaconAnalysisTest extends LinearOpMode {
//
//    @Override
//    public void runOp() throws InterruptedException {
//        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
//        params.vuforiaLicenseKey = "ATL6i4D/////AAAAGTkmmlOAlE6bi5wxY+GFDEYkiReJ4JoLKVqIj7L5JPEFanFxDXGWvNPh5QR4YboR1fVEnH7msYfNfuIiuARyXfZFlOWBYQ7PYL7s6zhTc7dDhxeF/HKTiiNUsnS2ahWhMbOyOQcwERpRwjTOONg1gcObtCJWVHHui3EphLOLOUFsumD5jQ4V7SkAnQ8MpIOHM8ntNJ0cwcO2TYGWp7rhYwaX5pywNiFcTHn4/9QVSLjnWnA+iN8jypntxolwNAsYILKDTS+8vRgbOnNQWK/RTkEHDZxV0We627e/AINsgGQzwKb0cITEhnQDStnZJPAxv6mVY4+ssmng7ezD/M+JkiqrZhmtlk5mHx5+MTC+anMs";
//        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
//
//        VuforiaLocalizer locale = ClassFactory.createVuforiaLocalizer(params);
//        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
//        locale.setFrameQueueCapacity(1);
//
//        VuforiaTrackables beacons = locale.loadTrackablesFromAsset("FTC_2016-17");
//        VuforiaTrackableDefaultListener gears = (VuforiaTrackableDefaultListener) beacons.get(3).getListener();
//
//        waitForStart();
//        beacons.activate();
//
//        while (!gears.isVisible()) {
//            delay(1);
//        }//while
//
//        while (opModeIsActive()) {
//            int beaconConfig = VortexUtils.NOT_VISIBLE;
//            while (beaconConfig == VortexUtils.NOT_VISIBLE) {
//                beaconConfig = VortexUtils.getJewelConfig(getImageFromFrame(locale.getFrameQueue().take(), PIXEL_FORMAT.RGB565), gears, locale.getCameraCalibration());
//            }//while
//
//            if (beaconConfig == VortexUtils.BEACON_RED_BLUE) {
//                Log.i("RED", "BLUE");
//            } else if (beaconConfig != VortexUtils.NOT_VISIBLE) {
//                Log.i("BLUE", "RED");
//            } else {
//                Log.i("BEAC", "== -1");
//            }//else
//
//            delay(500);
//        }//while
//    }//runOp
//
//
//    @Nullable
//    public static Image getImageFromFrame(VuforiaLocalizer.CloseableFrame frame, int format) {
//
//        long numImgs = frame.getNumImages();
//        for (int i = 0; i < numImgs; i++) {
//            if (frame.getImage(i).getFormat() == format) {
//                return frame.getImage(i);
//            }//if
//        }//for
//
//        return null;
//    }
//
//}