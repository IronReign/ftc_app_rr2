/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.TensorflowIntegration.GoldPos;

import java.nio.ByteBuffer;
import java.util.concurrent.BlockingQueue;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "TensorFlow Debug", group = "Linear Opmode")
@Config
public class TensorflowDebug extends LinearOpMode {

    // Declare OpMode members.


    // adjust these parameters to suit your needs
    public static int QUALITY = 25;
    public static double SCALE = 0.4;



    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        TensorflowIntegration tf = new TensorflowIntegration();
//        tf.tfInit(hardwareMap, telemetry);
        tf.initVuforia();
        BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue = tf.getInternalVuforia().getFrameQueue();

        while (opModeIsActive()) {
            FtcDashboard dashboard = FtcDashboard.getInstance();
//            GoldPos gp = tf.detect();
            GoldPos gp = GoldPos.ERROR1;
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("GoldTelemetry", gp.toString());

            dashboard.sendTelemetryPacket(packet);

            if (!frameQueue.isEmpty()) {
                long start = System.nanoTime();

                VuforiaLocalizer.CloseableFrame vuforiaFrame = null;
                try {
                    vuforiaFrame = frameQueue.peek();//
                } catch (Exception e) {
                    Thread.currentThread().interrupt();
                }

                if (vuforiaFrame == null) {
                    continue;
                }

                for (int i = 0; i < vuforiaFrame.getNumImages(); i++) {
                    Image image = vuforiaFrame.getImage(i);
                    if (image.getFormat() == PIXEL_FORMAT.RGB565) {
                        int imageWidth = image.getWidth(), imageHeight = image.getHeight();
                        ByteBuffer byteBuffer = image.getPixels();

                        Bitmap original = Bitmap.createBitmap(imageWidth, imageHeight, Bitmap.Config.RGB_565);
                        original.copyPixelsFromBuffer(byteBuffer);
                        Bitmap scaled = Bitmap.createScaledBitmap(original, (int) (SCALE * imageWidth), (int) (SCALE * imageHeight), false);

                        dashboard.setImageQuality(QUALITY);
                        dashboard.sendImage(scaled);
                    }
                }

                vuforiaFrame.close();

            }
        }
        tf.tfDisable();
    }
}
