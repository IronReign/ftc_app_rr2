/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.HINT;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.util.VisionUtils;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.util.VisionUtils.getImageFromFrame;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the tertiaryAuto or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Game_6832", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//  @Autonomous

public class Game_6832 extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    private Pose robot = new Pose();

    SoundPlayer deadShotSays = SoundPlayer.getInstance();

    private boolean active = true;
    boolean joystickDriveStarted = false;


    private int beaconConfig = 0;

    private boolean isBlue = false;
    private double pwrDamper = .33;
    private double pwrFwd = 0;
    private double pwrStf = 0;
    private double pwrRot = 0;

    private double vuPwr = 0;

    private ColorBlobDetector mDetector;

    Orientation angles;

    private int state = 0;
    private int autoStage = 0;
    private  int vuTestMode = 0;
    private boolean runAutonomous = true;
    private long autoTimer = 0;
    private long autoDelay = 0;
    private boolean[] buttonSavedStates = new boolean[11];
    boolean jewelMatches = false;



    //these are meant as short term testing variables, don't expect their usage
    //to be consistent across development sessions
    double testableDouble = robot.KpDrive;
    double testableHeading = 0;
    boolean testableDirection = true;

    private int a = 0; //collect (particle mode), manual cap lower (cap mode)
    private int b = 1; //eject (particle mode)
    private int x = 2; //launch (particle mode)
    private int y = 3; //spin up (particle mode), manual cap raise (cap mode)
    private int dpad_down = 4; //no function (particle mode), lowers cap to next lowest preset (cap mode)
    private int dpad_up = 5; //no function (particle mode), raises cap to next highest preset (cap mode)
    private int dpad_left = 6; //toggles between particle and cap mode (both modes)
    private int dpad_right = 7; //no function
    private int left_bumper = 8; //increment state down (always)
    private int right_bumper = 9; //increment state up (always)
    private int startBtn = 10; //toggle active (always)

    public VuforiaTrackables relicCodex;
    public int savedVuMarkCodex = 0;
    VuforiaTrackable relicTemplate;
    public int codexFlashStage = 0;
    public long codexFlashTimer = 0;
    VuforiaLocalizer locale;



    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(this.hardwareMap, isBlue);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        configureDashboard();

        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.vuforiaLicenseKey = RC.VUFORIA_LICENSE_KEY;
        params.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;

        locale = ClassFactory.createVuforiaLocalizer(params);
        locale.setFrameQueueCapacity(1);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);

        Vuforia.setHint (HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 1);

        relicCodex = locale.loadTrackablesFromAsset("RelicVuMark");
        relicCodex.get(0).setName("RelicTemplate");


        relicTemplate = relicCodex.get(0);

//        waitForStart(); //this is commented out but left here to document that we are still doing the functions that waitForStart() normally does, but needed to customize it.

        relicCodex.activate();
        robot.resetMotors(true);

        mDetector = new ColorBlobDetector();

        while(!isStarted()){    // Wait for the game to start (driver presses PLAY)
            synchronized (this) {
                try {
                    this.wait();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                }
            }

            stateSwitch();

            if(toggleAllowed(gamepad1.x,x)) {

                    isBlue = !isBlue;

            }
            if(toggleAllowed(gamepad1.dpad_down,dpad_down)){

                autoDelay--;
                if(autoDelay < 0) autoDelay = 15;

            }
            if(toggleAllowed(gamepad1.dpad_up, dpad_up)){

                autoDelay++;
                if(autoDelay>15) autoDelay = 0;

            }

            telemetry.addData("Status", "Initialized");
            telemetry.addData("Status", "Auto Delay: " + Long.toString(autoDelay) + "seconds");
            telemetry.addData("Status", "Side: " + getAlliance());
            telemetry.update();

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }




        runtime.reset();

        if(!runAutonomous){
            state = 1;
        }



        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.update();
            stateSwitch();
            if(active) {
                switch(state){
                    case 0: //code for tele-op control
                        joystickDrive();
                        break;
                    case 1: //this is the tertiaryAuto we use if our teamates can also go for the beacons more reliably than we can; scores 2 balls and pushes the cap ball, also parks on the center element
                        autonomous();
                        break;
                    case 2:
                        autonomous2();
                        break;
                    case 3:

//                        robot.PIDTune(robot.balancePID, toggleAllowed(gamepad1.dpad_up,dpad_up), toggleAllowed(gamepad1.dpad_down,dpad_down), toggleAllowed(gamepad1.y,y), toggleAllowed(gamepad1.a,a), toggleAllowed(gamepad1.x,x));
                        break;
                    case 4:
                        vuTest((VuforiaTrackableDefaultListener) relicTemplate.getListener(),500);
                        beaconConfig = VisionUtils.NOT_VISIBLE;
                        beaconConfig = VisionUtils.getBeaconConfig(getImageFromFrame(locale.getFrameQueue().take(), PIXEL_FORMAT.RGB565), (VuforiaTrackableDefaultListener) relicTemplate.getListener(), locale.getCameraCalibration());
                        if (beaconConfig == VisionUtils.BEACON_RED_BLUE) {
                            Log.i("RED", "BLUE");
                        } else if (beaconConfig != VisionUtils.NOT_VISIBLE) {
                            Log.i("BLUE", "RED");
                        } else {
                            Log.i("BEAC", "== -1");
                        }
                        break;
                    case 5: //provides data for forwards/backwards calibration
                        joystickDriveStarted = false;
                        if(robot.driveForward(false, 1, .5)) active = false;
                        break;
                    case 6: //provides data for left/right calibration
                        joystickDriveStarted = false;
                        if(robot.getAverageAbsTicks() < 2000){
                            robot.driveMixer(0,0,0);
                        }
                        else robot.driveMixer(0,0,0);
                        break;
                    case 7: //IMU demo mode
                        robot.MaintainHeading(gamepad1.x);
                        break;
                    case 8: //servo testing mode
                        robot.servoTester(toggleAllowed(gamepad1.dpad_up, dpad_up), toggleAllowed(gamepad1.y, y), toggleAllowed(gamepad1.a,a), toggleAllowed(gamepad1.dpad_down, dpad_down));
                    case 9:
                        break;
                    default:
                        robot.StopAll();
                        break;
                }
                robot.updateSensors();
            }
            else robot.StopAll();

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }


    public void vuTest(VuforiaTrackableDefaultListener beaconTarget, double distance){
        if (toggleAllowed(gamepad1.x, x)) {
            vuTestMode++;
            if (vuTestMode > 1) vuTestMode=0;
        }
        switch (vuTestMode) {
            case 0: //do nothing
                break;
            case 1: // drive to center of beacon target
                vuPwr = robot.driveToBeacon(beaconTarget,isBlue, beaconConfig,500, 0.8, false, false);
                break;

        }
    }
    public void demo(){
        robot.MaintainHeading(gamepad1.x);

    }


    public String getRelicCodexStr(){
        RelicRecoveryVuMark relicConfig = RelicRecoveryVuMark.from(relicTemplate);
        if(relicConfig != RelicRecoveryVuMark.UNKNOWN){
            if(relicConfig == RelicRecoveryVuMark.LEFT) return "left";
            else if(relicConfig == RelicRecoveryVuMark.RIGHT) return "right";
            else return "center";
        }
        return "unknown";
    }

    public int getRelicCodex(){
        RelicRecoveryVuMark relicConfig = RelicRecoveryVuMark.from(relicTemplate);
        if(relicConfig != RelicRecoveryVuMark.UNKNOWN){
            if(relicConfig == RelicRecoveryVuMark.LEFT) return 0;
            else if(relicConfig == RelicRecoveryVuMark.RIGHT) return 2;
            else return 1;
        }
        return 1;
    }

    public boolean flashRelicCodex(){
        switch (savedVuMarkCodex){
            case 0:
                switch (codexFlashStage){
                    case 0:
                        codexFlashTimer = futureTime(.5f);
                        robot.headLamp.setPower(0);
                        codexFlashStage++;
                        break;
                    case 1:
                        if(codexFlashTimer < System.nanoTime()) {
                            codexFlashTimer = futureTime(.15f);
                            robot.headLamp.setPower(1);
                            codexFlashStage++;
                        }
                        break;
                    case 2:
                        if(codexFlashTimer < System.nanoTime()){
                            robot.headLamp.setPower(0);
                            codexFlashTimer = futureTime(.5f);
                            codexFlashStage++;}
                        break;
                    case 3:
                        if(codexFlashTimer < System.nanoTime()){
                            robot.headLamp.setPower(1);
                            codexFlashStage = 0;
                            return true;
                        }
                        break;
                    default:
                        codexFlashStage = 0;
                        break;
                }
                break;
            case 1:
                switch (codexFlashStage){
                    case 0:
                        codexFlashTimer = futureTime(.5f);
                        robot.headLamp.setPower(0);
                        codexFlashStage++;
                        break;
                    case 1:
                        if(codexFlashTimer < System.nanoTime()) {
                            codexFlashTimer = futureTime(.15f);
                            robot.headLamp.setPower(1);
                            codexFlashStage++;
                        }
                        break;
                    case 2:
                        if(codexFlashTimer < System.nanoTime()){
                            robot.headLamp.setPower(0);
                            codexFlashTimer = futureTime(.15f);
                            codexFlashStage++;}
                        break;
                    case 3:
                        if(codexFlashTimer < System.nanoTime()) {
                            codexFlashTimer = futureTime(.15f);
                            robot.headLamp.setPower(1);
                            codexFlashStage++;
                        }
                        break;
                    case 4:
                        if(codexFlashTimer < System.nanoTime()){
                            robot.headLamp.setPower(0);
                            codexFlashTimer = futureTime(.5f);
                            codexFlashStage++;}
                        break;
                    case 5:
                        if(codexFlashTimer < System.nanoTime()){
                            robot.headLamp.setPower(1);
                            codexFlashStage = 0;
                            return true;
                        }
                        break;
                    default:
                        codexFlashStage = 0;
                        break;
                }
                break;
            case 2:
                switch (codexFlashStage){
                    case 0:
                        codexFlashTimer = futureTime(.5f);
                        robot.headLamp.setPower(0);
                        codexFlashStage++;
                        break;
                    case 1:
                        if(codexFlashTimer < System.nanoTime()) {
                            codexFlashTimer = futureTime(.15f);
                            robot.headLamp.setPower(1);
                            codexFlashStage++;
                        }
                        break;
                    case 2:
                        if(codexFlashTimer < System.nanoTime()){
                            robot.headLamp.setPower(0);
                            codexFlashTimer = futureTime(.15f);
                            codexFlashStage++;}
                        break;
                    case 3:
                        if(codexFlashTimer < System.nanoTime()) {
                            codexFlashTimer = futureTime(.15f);
                            robot.headLamp.setPower(1);
                            codexFlashStage++;
                        }
                        break;
                    case 4:
                        if(codexFlashTimer < System.nanoTime()){
                            robot.headLamp.setPower(0);
                            codexFlashTimer = futureTime(.15f);
                            codexFlashStage++;}
                        break;
                    case 5:
                        if(codexFlashTimer < System.nanoTime()) {
                            codexFlashTimer = futureTime(.15f);
                            robot.headLamp.setPower(1);
                            codexFlashStage++;
                        }
                        break;
                    case 6:
                        if(codexFlashTimer < System.nanoTime()){
                            robot.headLamp.setPower(0);
                            codexFlashTimer = futureTime(.5f);
                            codexFlashStage++;}
                        break;
                    case 7:
                        if(codexFlashTimer < System.nanoTime()){
                            robot.headLamp.setPower(1);
                            codexFlashStage = 0;
                            return true;
                        }
                        break;
                    default:
                        codexFlashStage = 0;
                        break;
                }
                break;

        }
        return false;
    }




    public void joystickDrive(){

        /*button indexes:
        0  = a
        1  = b
        2  = x
        3  = y
        4  = dpad_down
        5  = dpad_up
        6  = dpad_left
        7  = dpad_right
        8  = left bumper
        9  = right bumper
        10 = start button
        */

        if (!joystickDriveStarted) {
            robot.resetMotors(true);
            joystickDriveStarted = true;
        }
        pwrFwd = pwrDamper * gamepad1.left_stick_y;
        pwrStf = pwrDamper * gamepad1.left_stick_x;
        pwrRot = -pwrDamper * gamepad1.right_stick_x;
        robot.driveMixer(pwrFwd,pwrStf,pwrRot);

        if(gamepad1.right_trigger > 0.5)
            pwrDamper = 1;
        else
            pwrDamper = .33;

        if(gamepad1.a){
            robot.glyphSystem.lowerLift2();
        }

        if(gamepad1.y){
            robot.glyphSystem.raiseLift2();
        }


        if(toggleAllowed(gamepad1.b, b)){
            robot.glyphSystem.ToggleGrip();
        }

        if (gamepad1.dpad_down) robot.glyphSystem.goLiftMin();
        if (gamepad1.dpad_up) robot.glyphSystem.goLiftMax();
        if (gamepad1.dpad_right) robot.glyphSystem.goLiftStack();

//        degreeRot = -gamepad1.right_stick_x * 45; //hard right maps to 45 degree steering

//        if(toggleAllowed(gamepad1.y, y)){
////            robot.setKdDrive(robot.getKdDrive() + 10);
//        }
//
//        if(toggleAllowed(gamepad1.a, a)){
////            robot.setKdDrive(robot.getKdDrive() - 10);
//        }
//
//        if(toggleAllowed(gamepad1.dpad_up, dpad_up)){
//            robot.setKpDrive(robot.getKpDrive() + 0.005);
//        }
//
//        if(toggleAllowed(gamepad1.dpad_down, dpad_down)){
//            robot.setKpDrive(robot.getKpDrive() - 0.005);
//        }
//
//        if (!runDemo && !robot.isBalanceMode())
//            robot.driveMixer(pwrFwd, degreeRot);

    }

    public void autonomous(){
        switch(autoStage){
            case 0:
                autoTimer = futureTime(1.5f);
                robot.resetMotors(true);
                autoStage++;
            case 1: //scan vuforia target and deploy jewel arm
                //robot.jewel.lowerArm();
                if(autoTimer < System.nanoTime()) {
                    savedVuMarkCodex = getRelicCodex();

                    autoStage++;
                }
                break;
            case 2:
                if(robot.driveForward(false, .03, .25)){
                    jewelMatches = robot.doesJewelMatch(isBlue);
                    robot.resetMotors(true);
                    autoStage++;
                }
                break;
            case 3: //small turn to knock off jewel
                if ((isBlue && jewelMatches)||(!isBlue && !jewelMatches)){
                    if(robot.RotateIMU(10, 2.5)){
                        autoTimer = futureTime(1.5f);
                        autoStage++;
                        robot.resetMotors(true);
                    }
                }
                else{
                    if(robot.RotateIMU(350, 2.5)){
                        autoTimer = futureTime(1.5f);
                        autoStage++;
                        robot.resetMotors(true);
                    }
                }
                break;
            case 4:
                if(robot.driveForward(true, .03, .25)){
                    robot.resetMotors(true);
                    autoStage++;
                }
                break;
            case 5: //lift jewel arm
                robot.jewel.liftArm();
                if(autoTimer < System.nanoTime()) {
                    autoStage++;
                }
                break;
            case 6: //turn parallel to the wall
                if(isBlue){
                    if(robot.RotateIMU(270, 3.5)){
                        robot.resetMotors(true);
                        autoStage++;
                    }
                }
                else{
                    if(robot.RotateIMU(90, 3.5)){
                        robot.resetMotors(true);
                        autoStage++;
                    }
                }
                break;
            case 7: //drive off the balance stone
                if(robot.driveForward(false, .3, .5)) {
                    robot.resetMotors(true);
                    autoStage++;
                }
                break;
            case 8: //re-orient robot
                if(isBlue){
                    if(robot.RotateIMU(270, 1.0)){
                        robot.resetMotors(true);
                        autoStage++;
                    }
                }
                else{
                    if(robot.RotateIMU(90, 1.0)){
                        robot.resetMotors(true);
                        autoStage++;
                    }
                }
                break;
            case 9: //drive to proper crypto box column based on vuforia target
                switch (savedVuMarkCodex) {
                    case 0:
                        if(robot.driveForward(false, .5, .35)) {
                            robot.resetMotors(true);
                            autoStage++;
                        }
                        break;
                    case 1:
                        if(robot.driveForward(false, .75, .35)) {
                            robot.resetMotors(true);
                            autoStage++;
                        }
                        autoStage++;
                        break;
                    case 2:
                        if(robot.driveForward(false, 1.0, .35)) {
                            robot.resetMotors(true);
                            autoStage++;
                        }
                        autoStage++;
                        break;
                }
                break;
            case 10: //turn to crypto box
                if(isBlue){
                    if(robot.RotateIMU(325, 1.5)){
                        robot.resetMotors(true);
                        autoStage++;
                     }
                }
                else{
                    if(robot.RotateIMU(35, 1.5)){
                        robot.resetMotors(true);
                        autoStage++;
                    }
                }
                break;
            case 11: //deposit glyph
                if(robot.driveForward(false, 1.0, .50)) {
                    robot.resetMotors(true);
                    robot.glyphSystem.ReleaseGrip();
                    autoTimer = futureTime(1.5f);
                    autoStage++;
                }
                break;
            case 12:
                if(autoTimer < System.nanoTime()){
                    autoStage++;
                }
                break;
            case 13: //back away from crypto box
                if(robot.driveForward(true, .15, .50)){
                    robot.resetMotors(true);
                    autoStage++;
                }
                break;
            case 14:
                autoTimer = futureTime(1.5f);
                robot.glyphSystem.CloseGrip();
                autoStage++;
                break;
            case 15:
                if(autoTimer < System.nanoTime()){
                    autoStage++;
                }
                break;
            case 16:
                //back away from crypto box
                if(robot.driveForward(false, .25, .50)){
                    robot.resetMotors(true);
                    autoStage++;
                }
                break;
            case 17:
                //back away from crypto box
                if(robot.driveForward(true, .05, .30)){
                    robot.resetMotors(true);
                    autoStage++;
                }
                break;
            default:
                robot.resetMotors(true);
                autoStage = 0;
                active = false;
                state = 0;
                break;
        }
    }
    public void autonomous2 (){

        switch(autoStage){
            case 0: //deposit glyph
                if(robot.driveForward(false, 1.0, .50)) {
                    robot.resetMotors(true);
                    robot.glyphSystem.ReleaseGrip();
                    autoTimer = futureTime(1.5f);
                    autoStage++;
                }
                break;
            case 1:
                if(autoTimer < System.nanoTime()){
                    autoStage++;
                }
                break;
            case 2: //back away from crypto box
                if(robot.driveForward(true, .15, .50)){
                    robot.resetMotors(true);
                    autoStage++;
                }
                break;
            case 3:
                autoTimer = futureTime(1.5f);
                robot.glyphSystem.CloseGrip();
                autoStage++;
                break;
            case 4:
                if(autoTimer < System.nanoTime()){
                    autoStage++;
                }
                break;
            case 5:
                //back away from crypto box
                if(robot.driveForward(false, .25, .50)){
                    robot.resetMotors(true);
                    autoStage++;
                }
                break;
            case 6:
                //back away from crypto box
                if(robot.driveForward(true, .05, .30)){
                    robot.resetMotors(true);
                    autoStage++;
                }
                break;
            default:
                robot.resetMotors(true);
                autoStage = 0;
                active = false;
                state = 0;
                break;
        }
    }



    public void resetAuto(){
        autoStage = 0;
        autoTimer = 0;
        robot.ResetTPM();
    }


    public void stateSwitch() {

        /*button indexes:
        0  = a
        1  = b
        2  = x
        3  = y
        4  = dpad_down
        5  = dpad_up
        6  = dpad_left
        7  = dpad_right
        8  = left bumper
        9  = right bumper
        10 = start button
        */

        if (toggleAllowed(gamepad1.left_bumper, left_bumper)) {

            state--;
            if (state < 0) {
                state = 9;
            }
            robot.resetMotors(true);
            active = false;
            resetAuto();
            codexFlashStage = 0;
        }

        if (toggleAllowed(gamepad1.right_bumper, right_bumper)) {

            state++;
            if (state > 9) {
                state = 0;
            }
            robot.resetMotors(true);
            active = false;
            resetAuto();
            codexFlashStage = 0;
        }

        if (toggleAllowed(gamepad1.start, startBtn)) {
            robot.resetMotors(true);
            active = !active;
            codexFlashStage = 0;
        }
    }


    boolean toggleAllowed(boolean button, int buttonIndex)
    {

        /*button indexes:
        0  = a
        1  = b
        2  = x
        3  = y
        4  = dpad_down
        5  = dpad_up
        6  = dpad_left
        7  = dpad_right
        8  = left bumper
        9  = right bumper
        10 = start button
        */

        if (button) {
            if (!buttonSavedStates[buttonIndex])  { //we just pushed the button, and when we last looked at it, it was not pressed
                buttonSavedStates[buttonIndex] = true;
                return true;
            }
            //       else if(buttonCurrentState[buttonIndex] == buttonSavedStates[buttonIndex] && buttonCurrentState[buttonIndex]){
            else { //the button is pressed, but it was last time too - so ignore

                return false;
            }
        }

        buttonSavedStates[buttonIndex] = false; //not pressed, so remember that it is not
        return false; //not pressed

    }


    public String getAlliance(){
        if(isBlue)
            return "Blue";
        return "Red";
    }





    void configureDashboard() {
        // Configure the dashboard.

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = robot.imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            }
        });


        telemetry.addLine()
                .addData("active", new Func<String>() {
                    @Override public String value() {
                        return Boolean.toString(active);
                    }
                })
                .addData("state", new Func<String>() {
                    @Override public String value() {
                        return Integer.toString(state);
                    }
                })
                .addData("Servo Tester", new Func<String>() {
                    @Override public String value() {
                        return Integer.toString(robot.servoTesterPos);
                    }
                })
                .addData("servoJewel", new Func<String>() {
                    @Override public String value() {
                        return Integer.toString(robot.jewel.jewelPos);
                    }
                });
//        telemetry.addLine()
//                .addData("Kp", new Func<String>() {
//                    @Override public String value() {
//                        return "" + robot.getKpDrive();
//                    }
//                })
//                .addData("Kd", new Func<String>() {
//                    @Override public String value() {
//                        return "" + robot.getKdDrive();
//                    }
//                });
//
        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return robot.imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return robot.imu.getCalibrationStatus().toString();
                    }
                })
                .addData("Relic Codex", new Func<String>() {
                    @Override public String value() {
                        return getRelicCodexStr();
                    }
                })
                .addData("Relic Codex", new Func<String>() {
                    @Override public String value() {
                        return Integer.toString(savedVuMarkCodex);
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        //return formatAngle(angles.angleUnit, angles.firstAngle);
                        return Double.toString(robot.getHeading());
                    }
                })
                .addData("vuPwr", new Func<String>() {
                    @Override public String value() {
                        //return formatAngle(angles.angleUnit, angles.firstAngle);
                        return Double.toString(vuPwr);
                    }
                })
                .addData("vuDist", new Func<String>() {
                    @Override public String value() {
                        //return formatAngle(angles.angleUnit, angles.firstAngle);
                        return Double.toString(robot.getVuDepth());
                    }
                });
//                .addData("headingRaw", new Func<String>() {
//                    @Override public String value() {
//                        return formatAngle(angles.angleUnit, angles.firstAngle);
//
//                    }
//                })
//                .addData("headingOffset", new Func<String>() {
//                    @Override public String value() {
//                        return Double.toString(robot.offsetHeading);
//
//                    }
//                })
//
//                .addData("rollRaw", new Func<String>() {
//                    @Override public String value() {
//                        //return formatAngle(angles.angleUnit, angles.secondAngle);
//                        return Double.toString(robot.getRoll());
//                    }
//                })
//                .addData("pitchRaw", new Func<String>() {
//                    @Override public String value() {
//                        return formatAngle(angles.angleUnit, angles.thirdAngle);
//                    }
//                });
        telemetry.addLine()
                .addData("State", new Func<String>() {
                    @Override public String value() {
                        return String.valueOf(autoStage);
                    }
                })
//                .addData("TicksFL", new Func<String>() {
//                    @Override public String value() {
//                        return Long.toString(robot.motorFront.getCurrentPosition());
//                    }
//                })
//                .addData("TicksBL", new Func<String>() {
//                    @Override public String value() {
//                        return Long.toString(robot.motorBack.getCurrentPosition());
//                    }
//                })
                .addData("TicksAvg", new Func<String>() {
                    @Override public String value() {
                        return Long.toString(robot.getAverageTicks());
                    }
                });
        telemetry.addLine()

                .addData("PID Calc", new Func<String>() {
                    @Override public String value() {
                        return Double.toString(robot.drivePID.performPID() );
                    }
                })
                .addData("PID Err", new Func<String>() {
                    @Override public String value() {
                        return Double.toString(robot.drivePID.getError());
                    }
                });

        telemetry.addLine()
                .addData("DistRear", new Func<String>() {
                    @Override public String value() {
                        return String.valueOf(robot.beaconDistAft);
                    }
                })

                .addData("DistFore", new Func<String>() {
                    @Override public String value() {
                        return String.valueOf(robot.beaconDistFore);
                    }
                })
                .addData("ForeColor", new Func<String>() {
                    @Override public String value() {
                        return Long.toString(robot.beaconColor);
                    }
                });

    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    long futureTime(float seconds){
        return System.nanoTime() + (long) (seconds * 1e9);
    }
}
