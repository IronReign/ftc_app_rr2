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

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.vision.Viewpoint;
import org.firstinspires.ftc.teamcode.vision.GoldPos;
import org.firstinspires.ftc.teamcode.vision.VisionProvider;
import org.firstinspires.ftc.teamcode.vision.VisionProviders;
import org.firstinspires.ftc.teamcode.vision.dogecv.DogeCVFinalStep;

import static org.firstinspires.ftc.teamcode.util.VisionUtils.getJewelConfig;

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

@TeleOp(name="Game_6832", group="Challenge")  // @Autonomous(...) is the other common choice
//  @Autonomous
public class Game_6832 extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    private PoseBigWheel robot = new PoseBigWheel();

    private boolean active = true;
    private boolean joystickDriveStarted = false;

    private int state = 0;
    private boolean isBlue = false;

    //drive train control variables
    private double pwrDamper = 1;
    private double pwrFwd = 0;
    private double pwrStf = 0;
    private double pwrRot = 0;
    private double pwrFwdL = 0;
    private double pwrStfL = 0;
    private double pwrFwdR = 0;
    private double pwrStfR = 0;
    private double beaterDamper = .75;
    private boolean enableTank = false;
    private boolean bypassJoysticks = false;
    private long damperTimer = 0;
    private int direction = 1;  //-1 to reverse direction
    private int currTarget = 0;

    //staging and timer variables
    private int autoStage = 0;
    private int autoSetupStage = 0;
    private long autoTimer = 0;
    private long elbowTimer = 0;
    private long autoDelay = 0;

    //sensors/sensing-related variables
    private Orientation angles;

    //these are meant as short term testing variables, don't expect their usage
    //to be consistent across development sessions
    private double testableDouble = robot.kpDrive;
    private double testableHeading = 0;
    private boolean testableDirection = true;

    //values associated with the buttons in the toggleAllowed method
    private boolean[] buttonSavedStates = new boolean[16];
    private int a = 0; //lower glyph lift
    private int b = 1; //toggle grip/release on glyph
    private int x = 2; //no function
    private int y = 3; //raise glyph lift
    private int dpad_down = 4; //enable/disable ftcdash telemetry
    private int dpad_up = 5; //vision init/de-init
    private int dpad_left = 6; //vision provider switch
    private int dpad_right = 7; //switch viewpoint
    private int left_bumper = 8; //increment state down (always)
    private int right_bumper = 9; //increment state up (always)
    private int startBtn = 10; //toggle active (always)
    private int left_trigger = 11; //vision detection
    private int right_trigger = 12;
    private int back_button = 13;
    private int left_stick_button = 14;
    private int right_stick_button = 15; //sound player



    private boolean butY = false;
    private boolean butA = false;
    private boolean butX = false;
    private boolean butB = false;
    private int targetPos = 0;


    private VisionProvider vp;
    private int visionProviderState;
    private boolean visionProviderFinalized;
    private boolean enableTelemetry = false;
    private static final Class<? extends VisionProvider>[] visionProviders = VisionProviders.visionProviders;
    private static final Viewpoint viewpoint = Viewpoint.WEBCAM;
    private GoldPos initGoldPosTest = null;
    private int mineralState = 0;

    private int soundState = 0;
    private int soundID = -1;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(this.hardwareMap, isBlue);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        configureDashboard();

        // waitForStart();
        // this is commented out but left here to document that we are still doing the
        // functions that waitForStart() normally does, but needed to customize it.

        robot.resetMotors(true);

        visionProviderFinalized = false;

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
            if(toggleAllowed(gamepad1.a,a)){
                autoDelay--;
                if(autoDelay < 0) autoDelay = 15;
            }
            if(toggleAllowed(gamepad1.y, y)){
                autoDelay++;
                if(autoDelay>15) autoDelay = 0;
            }

            if(!visionProviderFinalized && toggleAllowed(gamepad1.dpad_left, dpad_left)){
                visionProviderState = (visionProviderState+1) % visionProviders.length; //switch vision provider
            }
            if (!visionProviderFinalized && toggleAllowed(gamepad1.dpad_up, dpad_up)){
                initialization_initVisionProvider(); //this is blocking
            } else if (visionProviderFinalized && toggleAllowed(gamepad1.dpad_up, dpad_up)) {
                initialization_deinitVisionProvider(); //also blocking, but should be very quick
            }
            if(!visionProviderFinalized && toggleAllowed(gamepad1.dpad_down, dpad_down)){
                enableTelemetry = !enableTelemetry; //enable/disable FtcDashboard telemetry
            }
            if(visionProviderFinalized && gamepad1.left_trigger > 0.3){
                GoldPos gp = vp.detect();
                if (gp != GoldPos.HOLD_STATE)
                    initGoldPosTest = gp;
                telemetry.addData("Vision", "Prep detection: %s%s", initGoldPosTest, gp==GoldPos.HOLD_STATE?" (HOLD_STATE)":"");
            }

            if(soundState == 0 && toggleAllowed(gamepad1.right_stick_button, right_stick_button)) {
                initialization_initSound();
            }

            telemetry.addData("Vision", "Backend: %s (%s)", visionProviders[visionProviderState].getSimpleName(), visionProviderFinalized ? "finalized" : System.currentTimeMillis()/500%2==0?"**NOT FINALIZED**":"  NOT FINALIZED  ");
            telemetry.addData("Vision", "FtcDashboard Telemetry: %s", enableTelemetry ? "Enabled" : "Disabled");
            telemetry.addData("Vision", "Viewpoint: %s", viewpoint);

            telemetry.addData("Sound", soundState == 0 ? "off" :
                                               soundState == 1 ? "on" :
                                               soundState == 2 ? "file not found" :
                                                                 "other");

            telemetry.addData("Status", "Initialized");
            telemetry.addData("Status", "Auto Delay: " + Long.toString(autoDelay) + "seconds");
            telemetry.addData("Status", "Side: " + getAlliance());
            telemetry.update();

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }



        if(vp == null) {
            initialization_initDummyVisionProvider(); //this is blocking
        }

        vp.reset();

        robot.superman.restart(.75);
        robot.collector.restart(.4, .5);


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.update();
            stateSwitch();
            if(active) {
                switch(state){
                    case 0: //code for tele-op control
                        joystickDrive();
                        break;
                    case 1: //autonomous that goes to opponent's crater
                        auto_depotSide();
                        break;
                    case 2: //autonomous that only samples
                        auto_depotSample();
                        break;
                    case 3: //autonomous that starts in our crater
                        auto_craterSide();
                        break;
                    case 4:
                        break;
                    case 5:
                        break;
                    case 6:
                        break;
                    case 7:
                        break;
                    case 8: //turn to IMU
                        demo();
                        break;
                    case 9:
                        break;
                    case 10:
                        break;
                    default:
                        robot.stopAll();
                        break;
                }
                robot.updateSensors();
            }
            else {
                robot.stopAll();
            }

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }

    private void initialization_initSound() {
        telemetry.addData("Please wait", "Initializing Sound");
        telemetry.update();
        soundID = hardwareMap.appContext.getResources().getIdentifier("gracious", "raw", hardwareMap.appContext.getPackageName());
        boolean success = SoundPlayer.getInstance().preload(hardwareMap.appContext, soundID);
        if (success)
            soundState = 1;
        else
            soundState = 2;
    }

    private void initialization_deinitVisionProvider() {
        telemetry.addData("Please wait","Deinitializing vision");
        telemetry.update();
        vp.shutdownVision();
        vp = null;
        visionProviderFinalized = false;
    }

    private void initialization_initVisionProvider() {
        try {
            telemetry.addData("Please wait","Initializing vision");
            telemetry.update();
            vp = visionProviders[visionProviderState].newInstance();
            vp.initializeVision(hardwareMap, telemetry, enableTelemetry, viewpoint);
        } catch (IllegalAccessException | InstantiationException e) {
            throw new RuntimeException(e);
        }
        visionProviderFinalized = true;
    }

    private void initialization_initDummyVisionProvider() {
        try {
            telemetry.addData("Please wait","Initializing vision");
            telemetry.update();
            vp = VisionProviders.defaultProvider.newInstance();
            vp.initializeVision(hardwareMap, telemetry, enableTelemetry, viewpoint);
        } catch (IllegalAccessException | InstantiationException e) {
            throw new RuntimeException(e);
        }
        visionProviderFinalized = true;
    }

    private void demo(){
        if(gamepad1.x)
            robot.maintainHeading(gamepad1.x);
    }

    private void auto_craterSide(){
        switch(autoStage){
            case 0:
                if (auto_setup())
                    autoStage++;
                break;
            case 1://turn to mineral
                switch(mineralState){
                    case 0://left
                        if(robot.rotateIMU(345,3)){
                            robot.resetMotors(true);
                            autoStage++;
                        }
                        break;
                    case 1://middle
                        autoStage++;
                        break;
                    case 2://right
                        if(robot.rotateIMU(15,3)){
                            robot.resetMotors(true);
                            autoStage++;
                        }
                        break;
                }
                break;
            case 2://move mineral
                switch(mineralState){
                    case 0://left
                        if(robot.driveForward(true, .880,.65)){
                            robot.resetMotors(true);
                            autoStage++;
                        }
                        break;
                    case 1://middle
                        if(robot.driveForward(true, .762,.65)){
                            robot.resetMotors(true);
                            autoStage++;
                        }
                        break;
                    case 2://right
                        if(robot.driveForward(true, .890,.65)){
                            robot.resetMotors(true);
                            autoStage++;
                        }
                        break;
                }
                break;
            case 3://move back
                switch(mineralState){
                    case 0://left
                        if(robot.driveForward(false, .880,.65)){
                            robot.resetMotors(true);
                            autoStage++;
                        }
                        break;
                    case 1://middle
                        if(robot.driveForward(false, .762,.65)){
                            robot.resetMotors(true);
                            autoStage++;
                        }
                        break;
                    case 2://right
                        if(robot.driveForward(false, .890,.65)){
                            robot.resetMotors(true);
                            autoStage++;
                        }
                        break;
                }
                break;
            case 4://turn parallel to minerals
                if(robot.rotateIMU(-90,3)){
                    robot.resetMotors(true);
                    autoStage++;
                }
                break;
            case 5://move to wall
                if(robot.driveForward(true, 1.3, .65)) {
                    robot.resetMotors(true);
                    autoStage++;
                }
                break;
            case 6://turn to depot
                if(robot.rotateIMU(225, 3)){
                    robot.resetMotors(true);
                    autoStage++;
                }
                break;
            case 7://move to depot
                if(robot.driveForward(true, 0.91, .65)) {
                    robot.resetMotors(true);
                    autoStage++;
                }
                break;
            case 8://start yeeting ducky
                //start collector and timer to yeet ducky
                autoTimer = futureTime(4);
                robot.collector.eject();
                autoStage++;
                break;
            case 9://stop yeeting ducky
                if(autoTimer<System.nanoTime()){
                    robot.collector.stopIntake();
                    robot.resetMotors(true);
                    autoStage++;
                }
                break;
            case 10://move backwards to crater
                if(robot.driveForward(false, 2, .65)) {
                    robot.collector.stopIntake();
                    robot.resetMotors(true);
                    autoStage++;
                }
            default:
                robot.resetMotors(true);
                autoStage = 0;
                active = false;
                state = 0;
                break;
        }
    }

    private void auto_depotSide(){
        switch(autoStage){
            case 0:
                if (auto_setup())
                    autoStage++;
                break;
            case 1://turn to mineral
                switch(mineralState){
                    case 0://left
                        if(robot.rotateIMU(39,3)){
                            robot.resetMotors(true);
                            autoSetupStage++;
                        }
                        break;
                    case 1://middle
                        autoSetupStage++;
                        break;
                    case 2://right
                        if(robot.rotateIMU(321,3)){
                            robot.resetMotors(true);
                            autoSetupStage++;
                        }
                        break;
                }
                break;
            case 2://move to mineral
                switch(mineralState){
                    case 0://left
                        if(robot.driveForward(true, .604,.65)){
                            robot.resetMotors(true);
                            autoSetupStage++;
                        }
                        break;
                    case 1://middle
                        if(robot.driveForward(true, .47,.65)){
                            robot.resetMotors(true);
                            autoSetupStage++;
                        }
                        break;
                    case 2://right
                        if(robot.driveForward(true, .604,.65)){
                            robot.resetMotors(true);
                            autoSetupStage++;
                        }
                        break;
                }
                break;
            case 3://turn to depot
                switch(mineralState){
                    case 0://left
                        if(robot.rotateIMU(345,3)){
                            robot.resetMotors(true);
                            autoStage++;
                        }
                        break;
                    case 1://middle
                        autoStage++;
                        break;
                    case 2://right
                        if(robot.rotateIMU(15,3)){
                            robot.resetMotors(true);
                            autoStage++;
                        }
                        break;
                }
                break;
            case 4://move to depot
                switch(mineralState){
                    case 0://left
                        if(robot.driveForward(true, .880,.65)){
                            //start collector and timer to yeet ducky
                            autoTimer = futureTime(4);
                            robot.collector.eject();
                            robot.resetMotors(true);
                            autoStage++;
                        }
                        break;
                    case 1://middle
                        if(robot.driveForward(true, .762,.65)){
                            //start collector and timer to yeet ducky
                            autoTimer = futureTime(4);
                            robot.collector.eject();
                            robot.resetMotors(true);
                            autoStage++;
                        }
                        break;
                    case 2://right
                        if(robot.driveForward(true, .890,.65)){
                            //start collector and timer to yeet ducky
                            autoTimer = futureTime(4);
                            robot.collector.eject();
                            robot.resetMotors(true);
                            autoStage++;
                        }
                        break;
                }
                break;
            case 5: //yeet ducky
                if(autoTimer<System.nanoTime()){
                    robot.collector.stopIntake();
                    robot.resetMotors(true);
                    autoStage++;
                }
                break;
            case 6: //turn to wall
                switch(mineralState){
                    case 0:
                        autoStage++;
                        break;
                    case 1:
                        if(robot.rotateIMU(225, 4)){
                            robot.resetMotors(true);
                            autoStage++;
                        }
                        break;
                    case 2:
                        if(robot.rotateIMU(225, 4)){
                            robot.resetMotors(true);
                            autoStage++;
                        }
                        break;
                }
                break;
            case 7: //move forward a lil
                switch(mineralState){
                    case 0://left
                        autoStage++;
                        break;
                    case 1://middle
                        if(robot.driveForward(false, .090, .65)){
                            robot.resetMotors(true);
                            autoStage++;
                        }
                        break;
                    case 2://right
                        if(robot.driveForward(false, .160,.65)){
                            robot.resetMotors(true);
                            autoStage++;
                        }
                        break;
                }
                break;
            case 8: //turn to crater
                if(robot.rotateIMU(303, 5)){
                    robot.resetMotors(true);
                    autoStage++;
                }
                break;
            case 9: //go to crater
                if(robot.driveForward(false, 1.05, .6)) {
                    robot.resetMotors(true);
                    autoStage++;
                }
                break;
            case 10: //turn to crater
                if(robot.rotateIMU(310, 1.5)){
                    robot.resetMotors(true);
                    autoStage++;
                }
                break;
            case 11: //go to crater
                if(robot.driveForward(false, .80, .6)) {
                    robot.resetMotors(true);
                    autoStage++;
                }
                break;
            case 12: //extend elbow to park
                robot.collector.setElbowTargetPos(robot.collector.posPreLatch+100);
                if(robot.collector.getElbowTargetPos() == robot.collector.posPreLatch+100){
                    robot.resetMotors(true);
                    autoStage++;
                }
            default:
                robot.resetMotors(true);
                autoStage = 0;
                active = false;
                state = 0;
                break;
        }
    }

    private void auto_depotSample(){
        switch(autoStage){
            case 0:
                if (auto_setup())
                    autoStage++;
                break;
            case 1://turn to mineral
                switch(mineralState){
                    case 0://left
                        if(robot.rotateIMU(39,3)){
                            robot.resetMotors(true);
                            autoSetupStage++;
                        }
                        break;
                    case 1://middle
                        autoSetupStage++;
                        break;
                    case 2://right
                        if(robot.rotateIMU(321,3)){
                            robot.resetMotors(true);
                            autoSetupStage++;
                        }
                        break;
                }
                break;
            case 2://move to mineral
                switch(mineralState){
                    case 0://left
                        if(robot.driveForward(true, .604,.65)){
                            robot.resetMotors(true);
                            autoSetupStage++;
                        }
                        break;
                    case 1://middle
                        if(robot.driveForward(true, .47,.65)){
                            robot.resetMotors(true);
                            autoSetupStage++;
                        }
                        break;
                    case 2://right
                        if(robot.driveForward(true, .604,.65)){
                            robot.resetMotors(true);
                            autoSetupStage++;
                        }
                        break;
                }
                break;
            case 3://turn to depot
                switch(mineralState){
                    case 0://left
                        if(robot.rotateIMU(345,3)){
                            robot.resetMotors(true);
                            autoStage++;
                        }
                        break;
                    case 1://middle
                        autoStage++;
                        break;
                    case 2://right
                        if(robot.rotateIMU(15,3)){
                            robot.resetMotors(true);
                            autoStage++;
                        }
                        break;
                }
                break;
            case 4://move to depot
                switch(mineralState){
                    case 0://left
                        if(robot.driveForward(true, .880,.65)){
                            //start collector and timer to yeet ducky
                            autoTimer = futureTime(4);
                            robot.collector.eject();
                            robot.resetMotors(true);
                            autoStage++;
                        }
                        break;
                    case 1://middle
                        if(robot.driveForward(true, .762,.65)){
                            //start collector and timer to yeet ducky
                            autoTimer = futureTime(4);
                            robot.collector.eject();
                            robot.resetMotors(true);
                            autoStage++;
                        }
                        break;
                    case 2://right
                        if(robot.driveForward(true, .890,.65)){
                            //start collector and timer to yeet ducky
                            autoTimer = futureTime(4);
                            robot.collector.eject();
                            robot.resetMotors(true);
                            autoStage++;
                        }
                        break;
                }
                break;
            case 5: //yeet ducky
                if(autoTimer<System.nanoTime()){
                    robot.collector.stopIntake();
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

    private boolean auto_sample() {
        //Turn on camera to see which is gold
        GoldPos gp = vp.detect();
        // Hold state lets us know that we haven't finished looping through detection
        if (gp != GoldPos.HOLD_STATE) {
            switch (gp) {
                case LEFT:
                    mineralState = 0;
                    break;
                case MIDDLE:
                    mineralState = 1;
                    break;
                case RIGHT:
                    mineralState = 2;
                    break;
                case NONE_FOUND:
                case ERROR1:
                case ERROR2:
                case ERROR3:
                default:
                    mineralState = 0;
                    break;
            }
            telemetry.addData("Vision Detection", "GoldPos: %s", gp.toString());
            vp.shutdownVision();
            return true;
        } else {
            telemetry.addData("Vision Detection", "HOLD_STATE (still looping through internally)");
            return false;
        }
    }

    private boolean auto_setup(){
        switch(autoSetupStage) {
            case 0:
                robot.setZeroHeading();
                robot.resetMotors(true);
                autoSetupStage++;
                break;
            case 1:
                if(robot.driveForward(true, .1,.5)){
                    robot.resetMotors(true);
                    autoSetupStage++;
                }
                break;
            case 2:
                /**Detach from lander**/
                autoSetupStage++;
                break;
            case 3:
                autoSetupStage++;
                break;
            case 4:
                if (auto_sample())
                    autoSetupStage++;
                break;
            case 5://turn to mineral
                switch(mineralState){
                    case 0://left
                        if(robot.rotateIMU(39,3)){
                            robot.resetMotors(true);
                            autoSetupStage++;
                        }
                        break;
                    case 1://middle
                        autoSetupStage++;
                        break;
                    case 2://right
                        if(robot.rotateIMU(321,3)){
                            robot.resetMotors(true);
                            autoSetupStage++;
                        }
                        break;
                }
                break;
            case 6://move to mineral
                switch(mineralState){
                    case 0://left
                        if(robot.driveForward(true, .604,.65)){
                            robot.resetMotors(true);
                            autoSetupStage++;
                        }
                        break;
                    case 1://middle
                        if(robot.driveForward(true, .47,.65)){
                            robot.resetMotors(true);
                            autoSetupStage++;
                        }
                        break;
                    case 2://right
                        if(robot.driveForward(true, .604,.65)){
                            robot.resetMotors(true);
                            autoSetupStage++;
                        }
                        break;
                }
                break;
            case 7:
                autoSetupStage++;
                break;
            case 8:
                autoSetupStage++;
                break;
            default:
                robot.resetMotors(true);
                autoSetupStage = 0;
                return true;
        }
        return false;
    }

    private void joystickDrive(){

        if (!joystickDriveStarted) {
            robot.resetMotors(true);
            joystickDriveStarted = true;
        }

        pwrFwd = direction * pwrDamper * gamepad1.left_stick_y;
        pwrStf = direction * pwrDamper * gamepad1.left_stick_x;
        pwrRot = -pwrDamper * .75 * gamepad1.right_stick_x;


        pwrFwdL = direction * pwrDamper * gamepad1.left_stick_y;
        pwrStfL = direction * pwrDamper * gamepad1.left_stick_x;

        pwrFwdR = direction * pwrDamper * gamepad1.right_stick_y;
        pwrStfR = direction * pwrDamper * gamepad1.right_stick_x;

        if (robot.getPitch()>45) {
            if (pwrDamper != .33) {
                pwrDamper = .33;
            } else
                pwrDamper = 1.0;
        }

        robot.driveMixerTank(pwrFwd, pwrRot);


        int standposition = 0;
        int restposition = 0;

        /*if(toggleAllowed(gamepad1.y, y)){
            if (robot.supermanMotor.getElbowCurrentPos() < standposition && robot.supermanMotor.getElbowTargetPos() < standposition) {
                robot.supermanMotor.setElbowTargetPos((int) Math.min(robot.supermanMotor.getElbowCurrentPos(), standposition));
                robot.supermanMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.supermanMotor.setElbowPwr(.80);
            }
        }else{
            if (robot.supermanMotor.getElbowCurrentPos() > restposition && robot.supermanMotor.getElbowTargetPos() > restposition) {
                robot.supermanMotor.setElbowTargetPos((int) Math.min(robot.supermanMotor.getElbowCurrentPos(), restposition));
                robot.supermanMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.supermanMotor.setElbowPwr(-.80);
            }
        }*/


        if(gamepad1.y){
            butY=true;
            butA = false;
            butX = false;
            butB = false;
        }
        if(gamepad1.a){
            butA=true;
            butY = false;
            butX = false;
            butB = false;
        }
        if(gamepad1.x){
            butX=true;
            butA = false;
            butY = false;
            butB = false;
        }
        if(gamepad1.b){
            butB=true;
            butA = false;
            butX = false;
            butY = false;
        }

        if(butA || butB || butX || butY) {
            robot.collector.setElbowTargetPos(1000);
            if (robot.collector.beltMid()) {
                if (butY) {//position for deposit
                    robot.superman.setTargetPosition(robot.superman.posDeposit);
                    targetPos = robot.superman.posDeposit;
                }
                if (butA) {  //position to collect minerals
                    robot.superman.setTargetPosition(robot.superman.posIntake);
                    targetPos = robot.superman.posIntake;
                }
                if (butX) { //target for pre-latching
                    robot.superman.setTargetPosition(robot.superman.posPreLatch);
                    robot.collector.setExtendABobPwr((robot.collector.extendMid+robot.collector.extendMax)/2);
                    targetPos = robot.superman.posPreLatch;
                }
                if (toggleAllowed(butB, b)) {
                    if (robot.collector.getElbowTargetPos() != robot.collector.posLatch) {
                        robot.superman.setTargetPosition(robot.superman.posLatch);
                        targetPos = robot.superman.posLatch;
                    } else {
                        robot.superman.setTargetPosition(robot.superman.posPostLatch);
                        targetPos = robot.superman.posPostLatch;
                    }
                }

                if (robot.superman.getTargetPosition() == targetPos && Math.abs(robot.collector.getElbowCurrentPos() - 1000) <20) {
                    if (butY) {//position for deposit
                        robot.collector.setElbowTargetPos(robot.collector.posDeposit);
                        butY = false;
                    }
                    if (butA) {  //position to collect minerals
                        robot.collector.setElbowTargetPos(robot.collector.posIntake);
                        butA = false;
                    }
                    if (butX) { //target for pre-latching
                        robot.collector.setElbowTargetPos(robot.collector.posPreLatch);
                        butX = false;
                    }
                    if (toggleAllowed(butB, b)) {
                        if (robot.collector.getElbowTargetPos() != robot.collector.posLatch) {
                            robot.collector.setElbowTargetPos(robot.collector.posLatch);
                        } else {
                            robot.collector.setElbowTargetPos(robot.collector.posPostLatch);
                        }
                        butB = false;
                    }
                }
            }
        }

        if(gamepad1.dpad_down){
            robot.superman.lower();
        }
        if(gamepad1.dpad_up){
            robot.superman.raise();
        }

        /*if(gamepad1.dpad_down){
            robot.collector.retract();
        }
        if(gamepad1.dpad_up){
            robot.collector.extend();
        }*/

        if(gamepad1.dpad_right){
            robot.collector.open();
        }
        if(gamepad1.dpad_left){
            robot.collector.close();
        }

        if(gamepad1.right_trigger > .5){
            robot.collector.collect();
        }

        else if(gamepad1.left_trigger > .5){
            robot.collector.eject();
        }
        currTarget = robot.collector.getExtendABobTargetPos();
        if(toggleAllowed(gamepad1.left_bumper, left_bumper)){
            if(currTarget == robot.collector.extendMid) {
                currTarget = robot.collector.extendMax;
            }
            else{
                currTarget = robot.collector.extendMid;
            }

        }
        if(currTarget >= 10){
            robot.collector.setExtendABobTargetPos(currTarget);
        }
        if(gamepad1.right_bumper){
            //robot.resetIMU();
            robot.collector.restart(.40, .5);
            robot.superman.restart(.75);
            robot.maintainHeading(gamepad1.right_bumper);
        }

        double triggers = gamepad1.left_trigger - gamepad1.right_trigger;

        if (triggers > 0.1)
            robot.collector.collect();
        else if (triggers < -0.1)
            robot.collector.eject();
        else
            robot.collector.stopIntake();

        if (soundState == 1 && toggleAllowed(gamepad1.right_stick_button, right_stick_button)) {
            SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, soundID);
        }

    }

    //the method that controls the main state of the robot; must be called in the main loop outside of the main switch
    private void stateSwitch() {
        if(!active) {
            if (toggleAllowed(gamepad1.left_bumper, left_bumper)) {

                state--;
                if (state < 0) {
                    state = 10;
                }
                robot.resetMotors(true);
                active = false;
            }

            if (toggleAllowed(gamepad1.right_bumper, right_bumper)) {

                state++;
                if (state > 10) {
                    state = 0;
                }
                robot.resetMotors(true);
                active = false;
            }

        }

        if (toggleAllowed(gamepad1.start, startBtn)) {
            robot.resetMotors(true);
            active = !active;
        }
    }


    //checks to see if a specific button should allow a toggle at any given time; needs a rework
    private boolean toggleAllowed(boolean button, int buttonIndex) {
        if (button) {
            if (!buttonSavedStates[buttonIndex])  { //we just pushed the button, and when we last looked at it, it was not pressed
                buttonSavedStates[buttonIndex] = true;
                return true;
            }
            else { //the button is pressed, but it was last time too - so ignore

                return false;
            }
        }

        buttonSavedStates[buttonIndex] = false; //not pressed, so remember that it is not
        return false; //not pressed

    }


    private String getAlliance(){
        if(isBlue)
            return "Blue";
        return "Red";
    }





    private void configureDashboard() {
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
                .addData("elbowA", new Func<String>() {
                    @Override public String value() {
                        return Boolean.toString(robot.collector.isActive());
                    }
                })
                .addData("elbowC", new Func<String>() {
                    @Override public String value() {
                        return Integer.toString(robot.collector.getElbowCurrentPos());
                    }
                })
                .addData("elbowC2", new Func<String>() {
                    @Override public String value() {
                        return Integer.toString(robot.collector.getElbowCurrentPos2());
                    }
                })
                .addData("elbowT", new Func<String>() {
                    @Override public String value() {
                        return Integer.toString(robot.collector.getElbowTargetPos());
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
                });
        telemetry.addLine()
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return robot.getRoll()+"";
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return robot.getPitch()+"";
                    }
                })
                .addData("yaw", new Func<String>() {
                    @Override public String value() {
                        return robot.getHeading()+"";
                    }
                });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return robot.imu.getSystemStatus().toShortString();
                    }
                })

                .addData("supermanPosition", new Func<String>(){
                    @Override public String value(){
                        return ""+robot.superman.getCurrentPosition();
                    }
                })

                .addData("extendABobPosition", new Func<String>(){
                    @Override public String value(){
                        return ""+robot.collector.getExtendABobCurrentPos();
                    }
                })

                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return robot.imu.getCalibrationStatus().toString();
                    }
                })
                .addData("servcPos", new Func<String>() {
                    @Override public String value() {
                        return robot.deposit.getPosition() + "";
                    }
                }).addData("autoStage", new Func<Integer>() {
                    @Override public Integer value() {
                        return autoStage;
                    }
                }).addData("mineratState", new Func<Integer>() {
                    @Override public Integer value() {
                        return mineralState;
                    }
                });
    }

    private long futureTime(float seconds){
        return System.nanoTime() + (long) (seconds * 1e9);
    }
}
