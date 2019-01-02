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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.HINT;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.util.VisionUtils;
import static org.firstinspires.ftc.teamcode.util.VisionUtils.getImageFromFrame;


/**
 * Demonstrates empty OpMode
 */
@Autonomous(name = "Concept: NullOp", group = "Concept")
//@Disabled
public class nullOpMode extends OpMode {

  private ElapsedTime runtime = new ElapsedTime();
  public VuforiaTrackables relicCodex;
  VuforiaTrackable relicTemplate;
  VuforiaLocalizer locale;

  int jewelConfig = 0;

  @Override
  public void init() {
    telemetry.addData("Status", "Initialized");

    VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
    params.vuforiaLicenseKey = RC.VUFORIA_LICENSE_KEY;

    params.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;

    locale = ClassFactory.createVuforiaLocalizer(params);
    locale.setFrameQueueCapacity(1);
    Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);

    Vuforia.setHint (HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 1);

    //set vuforia to look for assets from the Relic Recovery library
    relicCodex = locale.loadTrackablesFromAsset("RelicVuMark");
    relicCodex.get(0).setName("RelicTemplate");


    //relicTemplate = relicCodex.get(0);

//        waitForStart(); //this is commented out but left here to document that we are still doing the functions that waitForStart() normally does, but needed to customize it.

    //activate vuforia to start identifying targets/vuMarks

  }

  /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
  @Override
  public void init_loop() {


  }

  /*
   * This method will be called ONCE when start is pressed
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
   */
  @Override
  public void start() {
    relicCodex.activate();
   // runtime.reset();
  }

  /*
   * This method will be called repeatedly in a loop
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
   */
  @Override
  public void loop() {
    try {
      jewelConfig = VisionUtils.getJewelConfig(getImageFromFrame(locale.getFrameQueue().take(), PIXEL_FORMAT.RGB565), (VuforiaTrackableDefaultListener) relicTemplate.getListener(), locale.getCameraCalibration());
    }
    catch (Exception e){

    }
    telemetry.addData("Status", "Run Time: " + runtime.toString());
    telemetry.addData("Status", "Jewel Config: " + jewelConfig);
  }
}
