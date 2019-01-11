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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.vision.DummyVisionIntegration;
import org.firstinspires.ftc.teamcode.vision.GoldPos;
import org.firstinspires.ftc.teamcode.vision.OpenCVIntegration;
import org.firstinspires.ftc.teamcode.vision.TensorflowIntegration;
import org.firstinspires.ftc.teamcode.vision.VisionProvider;
import org.firstinspires.ftc.teamcode.vision.VisionProviders;

/**
 * Demonstrates empty OpMode
 */
@TeleOp(name = "VisionProvider Test", group = "Linear Opmode")
public class VisionProviderTest extends LinearOpMode {

    private static final Class<? extends VisionProvider>[] visionProviders = VisionProviders.visionProviders;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Configuration");
        telemetry.update();
        int visionProviderState = 0;
        boolean toggle = false;
        boolean visionProviderFinalized = false;
        VisionProvider vp = null;
        while (!isStarted()) {
            if (!visionProviderFinalized && gamepad1.dpad_left && !toggle) {
                toggle = true;
                visionProviderState++;
                if(visionProviderState == visionProviders.length)
                    visionProviderState = 0;
            } else if (!visionProviderFinalized && !gamepad1.dpad_left){
                toggle = false;
            } else if (!visionProviderFinalized && gamepad1.dpad_up) {
                try {
                    vp = visionProviders[visionProviderState].newInstance();
                    vp.initializeVision(hardwareMap, telemetry, true);
                } catch (IllegalAccessException | InstantiationException e) {
                    throw new RuntimeException(e);
                }
                visionProviderFinalized = true;
            }
            telemetry.addData("Status", "VisionBackend: %s (%s)", visionProviders[visionProviderState].getSimpleName(), visionProviderFinalized ? "finalized" : System.currentTimeMillis()/500%2==0?"**NOT FINALIZED**":"  NOT FINALIZED  ");
            telemetry.update();
        }
        telemetry.addData("Status", "Started");
        telemetry.update();
        if (vp == null) {
            vp = new DummyVisionIntegration();
            vp.initializeVision(hardwareMap, telemetry, true);
        }
        GoldPos gp = null;
        while (opModeIsActive()) {
            GoldPos newGp = vp.detect();
            if (newGp != GoldPos.HOLD_STATE)
                gp = newGp;
            telemetry.addData("VisionDetection", "%s", gp);
            telemetry.addData("HoldState", "%s", newGp == GoldPos.HOLD_STATE ? "YES" : "NO");
            telemetry.update();
        }
        vp.shutdownVision();
    }
}
