package org.firstinspires.ftc.teamcode.vision;

import android.database.sqlite.SQLiteClosable;
import android.graphics.Bitmap;
import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.RC;
import org.firstinspires.ftc.teamcode.util.VisionUtils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.List;
import java.util.Queue;

public class OpenCVIntegration implements VisionProvider {

    private VuforiaLocalizer vuforia;
    private Queue<VuforiaLocalizer.CloseableFrame> q;
    private int state = -3;
    private Mat mat;
    private List<MatOfPoint> contours;
    private Point lowest;

    private void initVuforia() {
        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = RC.VUFORIA_LICENSE_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        vuforia.setFrameQueueCapacity(1);
    }

    public void initializeVision(HardwareMap hardwareMap, Telemetry telemetry) {
        initVuforia();
        q = vuforia.getFrameQueue();
        state = -2;

    }

    public void shutdownVision() {}

    public GoldPos detect() {

//            if (q.isEmpty())
//                return GoldPos.HOLD_STATE;
            VuforiaLocalizer.CloseableFrame frame = null; //q.poll();
        try {
            frame = vuforia.getFrameQueue().take();
            Image img = VisionUtils.getImageFromFrame(frame, PIXEL_FORMAT.RGB565);
            if (img == null) return GoldPos.ERROR1;
            Bitmap bm = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
            bm.copyPixelsFromBuffer(img.getPixels());
            mat = VisionUtils.bitmapToMat(bm, CvType.CV_8UC3);

            RoverRuckusGripPipeline pipeline = new RoverRuckusGripPipeline();
            pipeline.process(mat);
            contours = pipeline.filterContoursOutput();

            if (contours.size() == 0)
                return GoldPos.NONE_FOUND;
            else return GoldPos.MIDDLE;
        }
            catch (Exception e) {

                Log.d("No Frame", e.toString());
                return GoldPos.ERROR2;

            }//if frame null


    }

    /* kvnote: I really have no clue how this is supposed to work
    public GoldPos detect() {
        if (state == -2) {
            if (q.isEmpty())
                return GoldPos.HOLD_STATE;
            VuforiaLocalizer.CloseableFrame frame = q.poll();
            Image img = VisionUtils.getImageFromFrame(frame, PIXEL_FORMAT.RGB565);
            if (img == null) return GoldPos.ERROR1;
            Bitmap bm = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
            bm.copyPixelsFromBuffer(img.getPixels());
            mat = VisionUtils.bitmapToMat(bm, CvType.CV_8UC3);
        } else if (state == -1) {
            RoverRuckusGripPipeline pipeline = new RoverRuckusGripPipeline();
            pipeline.process(mat);
            contours = pipeline.filterContoursOutput();
        } else if (state == 0) {
            if (contours.size() == 0)
                return GoldPos.NONE_FOUND;
            lowest = centroidish(contours.get(0));
        } else if (state < contours.size()) {
            Point centroid = centroidish(contours.get(state));
            if (lowest.y > centroid.y)
                lowest = centroid;
        } else if (state == contours.size()) {
            if (lowest.x < 320d / 3)
                return GoldPos.LEFT;
            else if (lowest.x < 640d / 3)
                return GoldPos.MIDDLE;
            else
                return GoldPos.RIGHT;
        } else {
            return GoldPos.ERROR2;
        }
        state++;
        return GoldPos.HOLD_STATE;
    }
    */

    private static Point centroidish(MatOfPoint matOfPoint) {
        Rect br = Imgproc.boundingRect(matOfPoint);
        return new Point(br.x + br.width/2,br.y + br.height/2);
    }
}
