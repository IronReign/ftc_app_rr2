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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.vision.Viewpoint;
import org.firstinspires.ftc.teamcode.vision.GoldPos;
import org.firstinspires.ftc.teamcode.vision.VisionProvider;
import org.firstinspires.ftc.teamcode.vision.VisionProviders;
import org.firstinspires.ftc.teamcode.vision.dogecv.DogeCVFinalStep;
import org.firstinspires.ftc.teamcode.vision.dogecv.DogeCVIntegration;

import java.util.Locale;

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

    //hi im testing something

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    private PoseBigWheel robot = new PoseBigWheel();

    SoundPlayer deadShotSays = SoundPlayer.getInstance();

    private boolean active = true;
    boolean joystickDriveStarted = false;
    public boolean suppressJoysticks = false;
    boolean balancing = false;

    private int state = 0;
    private boolean isBlue = false;
    private boolean relicMode = false;

    private boolean liftDeposit = false;
    private boolean liftVerticalDeposit = false;
    private boolean liftHome = false;
    private boolean liftCollect = false;

    private boolean retractRelic = false;
    private boolean extendRelic = false;
    private boolean placeRelic = false;

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

    //staging and timer variables
    private int autoStage = 0;
    private int autoSetupStage = 0;
    private long autoTimer = 0;
    private long elbowTimer = 0;
    private long autoDelay = 0;
    public int codexFlashStage = 0;

    //sensors/sensing-related variables
    Orientation angles;
    boolean vuActive = false;

    //these are meant as short term testing variables, don't expect their usage
    //to be consistent across development sessions
    double testableDouble = robot.kpDrive;
    double testableHeading = 0;
    boolean testableDirection = true;

    //values associated with the buttons in the toggleAllowed method
    private boolean[] buttonSavedStates = new boolean[14];
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



    boolean butY = false;
    boolean butA = false;
    boolean butX = false;
    boolean butB = false;
    int targetPos = 0;


    private VisionProvider vp;
    private int visionProviderState;
    private boolean visionProviderFinalized;
    private boolean enableTelemetry = false;
    private static final Class<? extends VisionProvider>[] visionProviders = VisionProviders.visionProviders;
    private int viewpoint = 2;
    private static final Viewpoint[] viewpoints = Viewpoint.values();
    private int dogeCvFinalStep = 1;
    private static final DogeCVFinalStep[] dogeCvFinalSteps = DogeCVFinalStep.values();
    private GoldPos initGoldPosTest = null;
    private int mineralState = 0;

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
                initializeVisionProvider(); //this is blocking
            } else if (visionProviderFinalized && toggleAllowed(gamepad1.dpad_up, dpad_up)) {
                deinitializeVisionProvider(); //also blocking, but should be very quick
            }
            if(!visionProviderFinalized && toggleAllowed(gamepad1.dpad_down, dpad_down)){
                enableTelemetry = !enableTelemetry; //enable/disable FtcDashboard telemetry
            }
            if(!visionProviderFinalized && toggleAllowed(gamepad1.dpad_right, dpad_right)) {
                viewpoint = (viewpoint+1) % viewpoints.length; //switch viewpoint
            }
            if(!visionProviderFinalized && toggleAllowed(gamepad1.b, b)){
                dogeCvFinalStep = (dogeCvFinalStep+1) % dogeCvFinalSteps.length; //switch dogecv final step
            }
            if(visionProviderFinalized && gamepad1.left_trigger > 0.3){
                GoldPos gp = vp.detect();
                if (gp != GoldPos.HOLD_STATE)
                    initGoldPosTest = gp;
                telemetry.addData("Vision", "Prep detection: %s%s", initGoldPosTest, gp==GoldPos.HOLD_STATE?" (HOLD_STATE)":"");
            }

            telemetry.addData("Vision", "Backend: %s (%s)", visionProviders[visionProviderState].getSimpleName(), visionProviderFinalized ? "finalized" : System.currentTimeMillis()/500%2==0?"**NOT FINALIZED**":"  NOT FINALIZED  ");
            telemetry.addData("Vision", "FtcDashboard Telemetry: %s", enableTelemetry ? "Enabled" : "Disabled");
            telemetry.addData("Vision", "Viewpoint: %s", viewpoints[viewpoint]);
            telemetry.addData("Vision", "DogeCVFInalStep: %s", dogeCvFinalSteps[dogeCvFinalStep]);


            telemetry.addData("Status", "Initialized");
            telemetry.addData("Status", "Auto Delay: " + Long.toString(autoDelay) + "seconds");
            telemetry.addData("Status", "Side: " + getAlliance());
            telemetry.update();

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }



        if(vp == null) {
            initializeVisionProvider(); //this is blocking
        }

        vp.reset();

        robot.superman.restart(.75);
        robot.superman.restart(.75);


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
                        autoSetup();
                        //autonomous();
                        break;
                    case 2:
                        //if(testDistnace())active=false;
                        auto1FromScratch();
                        break;
                    case 3:
                        if(testIMU())active=false;
                        //auto4();
                        break;
                    case 4: //IMU Following
                        demo();
                        break;
                    case 5: //provides data for forwards/backwards calibration
                        joystickDriveStarted = false;
                        if(robot.driveForward(true, .4, .35)) {
                            state = 0;
                            active = false;
                        }
                        break;
                    case 6: //provides data for left/right calibration
                        auto1();
                        break;
                    case 7: //IMU demo mode
//                        if(robot.jewel.retractArm())
//                            active = false;
//                        robot.relicArm.openGrip();
                        break;
                    case 8: //servo testing mode
//                        robot.servoTester(toggleAllowed(gamepad1.dpad_up, dpad_up), toggleAllowed(gamepad1.y, y), toggleAllowed(gamepad1.a,a), toggleAllowed(gamepad1.dpad_down, dpad_down));
//                        robot.relicArm.closeGrip();
                        if(gamepad1.x)
                            robot.maintainHeading(gamepad1.x);
                        break;
                    case 9:
                        //autonomous3();
                        break;
                    case 10: //vision testing
//                        autonomous3();
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

    private void deinitializeVisionProvider() {
        telemetry.addData("Please wait","Deinitializing vision");
        telemetry.update();
        vp.shutdownVision();
        vp = null;
    }

    private void initializeVisionProvider() {
        try {
            telemetry.addData("Please wait","Initializing vision");
            telemetry.update();
            vp = visionProviders[visionProviderState].newInstance();
            vp.initializeVision(hardwareMap, telemetry, enableTelemetry, viewpoints[viewpoint]);
            if (vp instanceof DogeCVIntegration)
                ((DogeCVIntegration) vp).setDogeCVFinalStep(dogeCvFinalSteps[dogeCvFinalStep]);
        } catch (IllegalAccessException | InstantiationException e) {
            throw new RuntimeException(e);
        }
        visionProviderFinalized = true;
    }


    public void demo(){
        //robot.glyphSystem.tiltPhoneDown();
        if(gamepad1.x){
            robot.maintainHeading(gamepad1.x);
        }
        /*
        if(gamepad1.y) {
            robot.driveToTargetVu(beaconTarget, isBlue, 0, distance, .5, true, false);
        }


        if(gamepad1.a) {
            robot.driveToTargetVu(beaconTarget, isBlue, 0, distance, .5, false, false);
        }
        */
    }

    public boolean testDistnace(){
        switch(autoSetupStage){
            case 0:
                robot.setZeroHeading();
                robot.resetMotors(true);
                autoSetupStage++;
                break;
            case 1:
                if(robot.driveForward(false, .5, .5)){
                    robot.resetMotors(true);
                    autoSetupStage++;
                    return  true;
                }
                break;
            default:
                robot.resetMotors(true);
                autoSetupStage = 0;
                return true;
        }
        return false;
    }

    public boolean testIMU(){
        switch(autoSetupStage){
            case 0:
                robot.setZeroHeading();
                robot.resetMotors(true);
                autoSetupStage++;
                break;
            case 1:
                if(robot.rotateIMU(90, 3 )){
                    robot.resetMotors(true);
                    autoSetupStage++;
                    return  true;
                }
                break;
            default:
                robot.resetMotors(true);
                autoSetupStage = 0;
                return true;
        }
        return false;
    }

    public boolean turnMineral(){
        switch(mineralState){
            case 0:
                if(robot.rotateIMU(30, 2)) return true;
                break;
            case 1:
                if(robot.rotateIMU(0, 1.5)) return true;
                break;
            case 2:
                if(robot.rotateIMU(330, 2)) return true;
                break;
            default:
                return true;
        }
        return false;
    }

    public boolean turnDepot(){
        switch(mineralState){
            case 0:
                if(robot.rotateIMU(-90, 3)) return true;
                break;
            case 1:
                if(robot.rotateIMU(0, 1.5)) return true;
                break;
            case 2:
                if(robot.rotateIMU(20, 3)) return true;
                break;
            default:
                return true;
        }
        return false;

    }

    public void auto1FromScratch(){
        switch(autoStage){
            case 0:
                robot.setZeroHeading();
                robot.resetMotors(true);
                autoStage++;
                break;
            case 1:
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
                    autoStage++;
                } else {
                    telemetry.addData("Vision Detection", "HOLD_STATE (still looping through internally)");
                }
                break;
            case 2://turn to mineral
                switch(mineralState){
                    case 0://left
                        if(robot.rotateIMU(39,3)){
                            robot.resetMotors(true);
                            autoStage++;
                        }
                        break;
                    case 1://middle
                        autoStage++;
                        break;
                    case 2://right
                        if(robot.rotateIMU(321,3)){
                            robot.resetMotors(true);
                            autoStage++;
                        }
                        break;
                }
                break;
            case 3://move to mineral
                switch(mineralState){
                    case 0://left
                        if(robot.driveForward(true, .604,.65)){
                            robot.resetMotors(true);
                            autoStage++;
                        }
                        break;
                    case 1://middle
                        if(robot.driveForward(true, .47,.65)){
                            robot.resetMotors(true);
                            autoStage++;
                        }
                        break;
                    case 2://right
                        if(robot.driveForward(true, .604,.65)){
                            robot.resetMotors(true);
                            autoStage++;
                        }
                        break;
                }
                break;
            case 4://turn to depot
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
            case 5://move to depot
                switch(mineralState){
                    case 0://left
                        if(robot.driveForward(true, .890,.65)){
                            //start collector and timer to yeet ducky
                            autoTimer = futureTime(4);
                            robot.collector.collect();
                            robot.resetMotors(true);
                            autoStage++;
                        }
                        break;
                    case 1://middle
                        if(robot.driveForward(true, .762,.65)){
                            //start collector and timer to yeet ducky
                            autoTimer = futureTime(4);
                            robot.collector.collect();
                            robot.resetMotors(true);
                            autoStage++;
                        }
                        break;
                    case 2://right
                        if(robot.driveForward(true, .890,.65)){
                            //start collector and timer to yeet ducky
                            autoTimer = futureTime(4);
                            robot.collector.collect();
                            robot.resetMotors(true);
                            autoStage++;
                        }
                        break;
                }
                break;
            case 6:
                //yeet ducky
                if(autoTimer<System.nanoTime()){
                    robot.collector.stopIntake();
                    robot.resetMotors(true);
                    autoStage++;
                }
                break;
            case 7:
                //turn to wall
                if(robot.rotateIMU(225, 4)){
                    robot.resetMotors(true);
                    autoStage++;
                }
                break;
            case 8:
                //move forward a lil
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
            case 9:
                //turn to crater
                if(robot.rotateIMU(310, 5)){
                    robot.resetMotors(true);
                    autoStage++;
                }
                break;
            case 10:
                //go to crater
                if(robot.driveForward(false, 2.7, .6)) {
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




    public boolean autoSetup(){
        switch(autoSetupStage) {
            case 0:
                robot.setZeroHeading();
                robot.resetMotors(true);
                autoSetupStage++;
                break;
            case 1:
                /**Detach from lander**/
                autoSetupStage++;
                break;
            case 2:
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
                            mineralState = 1;
                            break;
                        case ERROR1:
                        case ERROR2:
                        case ERROR3:
                        default:
                            mineralState = 1;
                            break;
                    }
                    telemetry.addData("Vision Detection", "GoldPos: %s", gp.toString());
                    vp.shutdownVision();
                    autoSetupStage++;
                } else {
                    telemetry.addData("Vision Detection", "HOLD_STATE (still looping through internally)");
                }
                autoSetupStage++;
                break;
            case 3:
                if(turnMineral()){
                    robot.resetMotors(true);
                    autoSetupStage++;
                }
                break;
            case 4:
                if(mineralState == 1) {
                    if (robot.driveForward(true, .65, .65)) {
                        robot.resetMotors(true);
                        autoSetupStage++;
                    }
                }
                else{
                    if (robot.driveForward(true, .85, .65)) {
                        robot.resetMotors(true);
                        autoSetupStage++;
                    }
                }
                break;
            case 5:
                /**Driving to remove gold off pedastal**/
                if(turnDepot()){
                    robot.resetMotors(true);
                    autoSetupStage++;
                }
                break;
            case 6:
                if(mineralState == 1) {
                    if (robot.driveForward(true, .85, .65)) {
                        robot.resetMotors(true);
                        autoSetupStage++;
                    }
                }
                if(mineralState == 0) {
                    if (robot.driveForward(true, .65, .65)) {
                        robot.resetMotors(true);
                        autoSetupStage++;
                    }
                }
                else{
                    if (robot.driveForward(true, .95, .65)) {
                        robot.resetMotors(true);
                        autoSetupStage++;
                    }
                }break;
            case 7:
                /**Correct appropriate distance to drive to approach center of the mineral**/
                autoTimer = futureTime(1);
                robot.collector.collect();
                autoSetupStage++;
                break;
            case 8:
                if(autoTimer < System.nanoTime()){
                    robot.collector.stopIntake();
                    autoSetupStage++;
                }
                /**Rotate to face forward**/
//                autoSetupStage++;
                break;
            case 9:
                autoSetupStage++;
                break;
            case 10:
                autoSetupStage++;
                break;
            case 11:
                autoSetupStage++;
                break;
            case 12:
                autoSetupStage++;
                break;
            case 13:
                autoSetupStage++;
                break;
            case 14:
                autoSetupStage++;
                break;
            case 15:
                autoSetupStage++;
                break;
            case 16:
                autoSetupStage++;
                break;
            case 17:
                autoSetupStage++;
                break;
            default:
                robot.resetMotors(true);
//                autoSetupStage = 0;
                return true;
        }
        return false;
    }

    private void auto1() { //Starts depot side and end opposite crater
        switch (autoStage) {
            case 0:
                //robot.setZeroHeading();
                //robot.resetMotors(true);
                autoStage++;
                break;
            case 1:
                if(autoSetup()){
                    autoStage++;
                }
                break;
            case 2:
                //Drive into depot
                if(robot.rotateIMU(135, 5)){
                    robot.resetMotors(true);
                    autoStage++;
                }
                break;
            case 3:
                //Drop ducky into depot
                /*if(robot.collector.setElbowTargetPos()){
                    robot.resetMotors(true);
                    autoStage++;
                }*/
                if(robot.driveForward(true, 2.7, .6))
                autoStage++;
                break;
            case 4:
                //Retract arm
                /*if(robot.collector.moveTo(rest position)){
                    robot.resetMotors(true);
                    autoStage++;
                }*/
                autoStage++;
                break;
            /*case 5:
                //Drive backward to safe distance from wall
                if(robot.driveForward(true, .3, .7)){
                    robot.resetMotors(true);
                    autoStage++;
                }
                break;
            case 6:
                //rotate to be parallel with wall
                if(robot.rotateIMU(-135, 3)){
                    robot.resetMotors(true);
                    autoStage++;
                }
                break;
            case 7:
                //drive to crater area
                if(robot.driveForward(false, 1, .75)){
                    robot.resetMotors(true);
                    autoStage++;
                }
                break;
            case 8:
                //extend into depot to prepare for start of match and get parking points
                /*if(robot.collector.moveTo(straightposition)){
                    robot.resetMotors(true);
                    autoStage++;
                }
                autoStage++;
                break;*/
            default:
                robot.resetMotors(true);
                autoStage = 0;
                active = false;
                state = 0;
                break;

        }
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


        if (balancing) { //balance with a simple drive forward from edge of stone

            if(robot.driveForward(true, .42, .45)){
                suppressJoysticks=false;
                robot.resetMotors(true);
                balancing = false;
                }
        }
            else {
                suppressJoysticks = false;

            }


        pwrFwd = direction * pwrDamper * gamepad1.left_stick_y;
        pwrStf = direction * pwrDamper * gamepad1.left_stick_x;
        pwrRot = -pwrDamper * .75 * gamepad1.right_stick_x;


//        pwrRot += .33 * (gamepad1.right_trigger - gamepad1.left_trigger);

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

        if (!suppressJoysticks) {
            robot.driveMixerTank(pwrFwd, pwrRot);
        }


//        if(robot.glyphSystem.getMotorLiftPosition() <= 2500) {
//            robot.glyphSystem.setMotorLeft(gamepad2.left_stick_y*beaterDamper);
//            robot.glyphSystem.setMotorRight(-gamepad2.left_stick_y*beaterDamper);
//        }

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


        /*if(toggleAllowed(gamepad1.left_bumper, left_bumper)){
            supermanTester = !supermanTester;
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
            if (robot.collector.beltMid()) {
                if (butY) {//position for deposit
                    robot.collector.setElbowTargetPos(robot.collector.posDeposit);
                    robot.superman.setTargetPosition(robot.superman.posDeposit);
                    targetPos = robot.collector.posDeposit;
                }
                if (butA) {  //position to collect minerals
                    robot.collector.setElbowTargetPos(robot.collector.posIntake);
                    robot.superman.setTargetPosition(robot.superman.posIntake);
                    targetPos = robot.collector.posIntake;
                }
                if (butX) { //target for pre-latching
                    robot.collector.setElbowTargetPos(robot.collector.posPreLatch);
                    robot.superman.setTargetPosition(robot.superman.posPreLatch);
                    targetPos = robot.collector.posPreLatch;
                }
                if (toggleAllowed(butB, b)) {
                    if (robot.collector.getElbowTargetPos() != robot.collector.posLatch) {
                        robot.collector.setElbowTargetPos(robot.collector.posLatch);
                        robot.superman.setTargetPosition(robot.superman.posLatch);
                        targetPos = robot.collector.posLatch;
                    } else {
                        robot.collector.setElbowTargetPos(robot.collector.posPostLatch);
                        robot.superman.setTargetPosition(robot.superman.posPostLatch);
                        targetPos = robot.collector.posPostLatch;
                    }
                }

                if (robot.collector.getElbowCurrentPos() == targetPos) {
                    butB = false;
                    butA = false;
                    butX = false;
                    butY = false;
                }
            }
        }

        /**
        if(butY==true ||butB==true ||butX==true ||butA==true ){
            switch(beltPos){
                case 0:
                    robot.collector.setExtendABobTargetPos(robot.collector.extendMin);
                    break;
                case 1:
                    robot.collector.setExtendABobTargetPos(robot.collector.extendMid);
                    if(robot.collector.getExtendABobCurrentPos()==robot.collector.extendMid){
                        if(butY){//y is deposit
                            robot.collector.setElbowTargetPos(robot.collector.posDeposit);
                            robot.superman.setTargetPosition(robot.superman.posDeposit);
                            targetPos = robot.collector.posDeposit;
                        }
                        if(butA){
                            robot.collector.setElbowTargetPos(robot.collector.posIntake);
                            robot.superman.setTargetPosition(robot.superman.posIntake);
                            targetPos = robot.collector.posIntake;
                        }
                        if(butX){
                            robot.collector.setElbowTargetPos(robot.collector.posPreLatch);
                            robot.superman.setTargetPosition(robot.superman.posPreLatch);
                            targetPos = robot.collector.posPreLatch;
                        }
                        if(toggleAllowed(butB,b)){
                            if(robot.collector.getElbowTargetPos()!=robot.collector.posLatch) {
                                robot.collector.setElbowTargetPos(robot.collector.posLatch);
                                robot.superman.setTargetPosition(robot.superman.posLatch);
                                targetPos = robot.collector.posLatch;
                            }
                            else {
                                robot.collector.setElbowTargetPos(robot.collector.posPostLatch);
                                robot.superman.setTargetPosition(robot.superman.posPostLatch);
                                targetPos = robot.collector.posPostLatch;
                            }
                        }

                        if(robot.collector.getElbowCurrentPos()==targetPos){
                            beltPos++;
                        }
                    }
                    break;
                case 2:
                    robot.collector.setExtendABobTargetPos(robot.collector.extendMin);
                    if(robot.collector.getExtendABobCurrentPos()==robot.collector.extendMid) {
                        butY = false;
                        butX = false;
                        butA = false;
                        butB = false;
                    }
                    break;

        }
        }**/



//        if(gamepad1.dpad_down){
//            robot.superman.lower();
//        }
//        if(gamepad1.dpad_up){
//            robot.superman.raise();
//        }

        if(gamepad1.dpad_down){
            robot.collector.retract();
        }
        if(gamepad1.dpad_up){
            robot.collector.extend();
        }

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

        if(gamepad1.left_bumper){
            robot.collector.setExtendABobTargetPos(robot.collector.extendMid);
        }
        if(gamepad1.right_bumper){
            //robot.resetIMU();
            robot.collector.restart(1, .5);
            robot.superman.restart(.75);
            robot.maintainHeading(gamepad1.right_bumper);
        }


        /*
        if(!supermanTester){
            if(gamepad1.y) robot.collector.raise();
            if(gamepad1.a) robot.collector.lower();
            if(gamepad1.x) robot.collector.kill();
            if(gamepad1.b) robot.collector.restart(.5);
            if(gamepad1.dpad_up) robot.collector.setElbowTargetPos(robot.collector.posIntake);
            if(gamepad1.dpad_down) robot.collector.setElbowTargetPos(robot.collector.posLatch);
            if(gamepad1.right_bumper) tf.tfDisable();
            if(gamepad1.dpad_right) telemetry.addData("TF Detection", "%s", tf.detect());
        }else{
            if(gamepad1.y) robot.superman.raise();
            if(gamepad1.a) robot.superman.lower();
            if(gamepad1.x) robot.superman.kill();
            if(gamepad1.b) robot.superman.restart(.75);
            if(gamepad1.dpad_up) robot.superman.setElbowTargetPos(robot.collector.posIntake);
            if(gamepad1.dpad_down) robot.superman.setElbowTargetPos(robot.collector.posLatch);
            if(Math.abs(gamepad1.left_trigger)>.5) robot.collector.collect();
            else if(Math.abs(gamepad1.right_trigger)>.5) robot.collector.eject();
            else robot.collector.stopIntake();
        }*/





        if(false){//!relicMode) {

            if(toggleAllowed(gamepad1.b, b)){
                //robot.glyphSystem.toggleBottomGrip();
            }



            if (toggleAllowed(gamepad1.left_bumper, left_bumper)) {
//                robot.glyphSystem.tiltPhoneUp();
                if (direction == 1) {
                    relicMode = false;
                    pwrDamper = .5;
                    direction = -1;
                   // robot.ledSystem.pinkPos();
                    liftVerticalDeposit = false;
                    liftDeposit = true;
                    liftHome = false;
                    liftCollect = false;
                } else {
                    relicMode = false;
                    relicMode = false;
                    direction = 1;
                    pwrDamper = 1.0;
                    if (isBlue) {
                        //robot.ledSystem.bluePos();
                    } else
                       // robot.ledSystem.redPos();
                    liftVerticalDeposit = false;
                    liftDeposit = false;
                    liftHome = false;
                    liftCollect = true;
                }
            }
            if (toggleAllowed(gamepad1.a, a)) {
                //robot.glyphSystem.toggleBelt(direction < 0);
            }


            if (toggleAllowed(gamepad1.x, x)) {
                //robot.glyphSystem.toggleGrip();
            }

            if(gamepad1.dpad_up) {
                if (direction > 0) {
//                    if (gamepad1.dpad_up) {
//                        robot.glyphSystem.tiltPhoneUp();
                        //robot.glyphSystem.raiseLift2();
//                    }
                }
                else {
//                    robot.glyphSystem.tiltPhoneUp();
                    liftVerticalDeposit = true;
                    liftDeposit = false;
                    liftHome = false;
                    liftCollect = false;
                }
            }
            else if (gamepad1.dpad_down) {
//                robot.glyphSystem.tiltPhoneUp();
                //robot.glyphSystem.lowerLift2();
            }
            else {
                //robot.glyphSystem.stopBelt();
            }
        }

    }

 /**
    public void resetAuto(){
        autoStage = 0;
        autoTimer = 0;
        robot.resetTPM();
    }
**/

    //the method that controls the main state of the robot; must be called in the main loop outside of the main switch
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

        if(!active) {
            if (toggleAllowed(gamepad1.left_bumper, left_bumper)) {

                state--;
                if (state < 0) {
                    state = 10;
                }
                robot.resetMotors(true);
                active = false;
                //resetAuto();
                codexFlashStage = 0;
            }

            if (toggleAllowed(gamepad1.right_bumper, right_bumper)) {

                state++;
                if (state > 10) {
                    state = 0;
                }
                robot.resetMotors(true);
                active = false;
                //resetAuto();
                codexFlashStage = 0;
            }

        }

        if (toggleAllowed(gamepad1.start, startBtn)) {
            robot.resetMotors(true);
            active = !active;
            codexFlashStage = 0;
        }
    }


    //checks to see if a specific button should allow a toggle at any given time; needs a rework
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

//                .addData("Servo Tester", new Func<String>() {
//                    @Override public String value() {
//                        return Integer.toString(robot.servoTesterPos);
//                    }
//                })


//                .addData("servoJewelExtender", new Func<String>() {
//                    @Override public String value() {
//                        return Integer.toString(robot.jewel.jewelPos);
//                    }
//                });
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
//                .addData("phone pos", new Func<String>() {
//                    @Override public String value() {
//                        return Integer.toString(robot.glyphSystem.maintainPhoneTilt());
//                    }
//                })
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
//                .addData("Jewel Red", new Func<String>() {
//                    @Override public String value() {
//                        return "" + robot.colorJewel.red();
//                    }
//                })
//
//                .addData("Jewel Blue", new Func<String>() {
//                    @Override public String value() {
//                        return "" + robot.colorJewel.blue();
//                    }
//                })

//                .addData("Relic Codex", new Func<String>() {
//                    @Override public String value() {
//                        return getRelicCodexStr();
//                    }
//                })
//                .addData("Relic Codex", new Func<String>() {
//                    @Override public String value() {
//                        return Integer.toString(savedVuMarkCodex);
//                    }
//                });

//        telemetry.addLine()
//                .addData("heading", new Func<String>() {
//                    @Override public String value() {
//                        //return formatAngle(angles.angleUnit, angles.firstAngle);
//                        return Double.toString(robot.getHeading());
//                    }
//                })
//                .addData("pitch", new Func<String>() {
//                    @Override public String value() {
//                        //return formatAngle(angles.angleUnit, angles.firstAngle);
//                        return Double.toString(robot.getPitch());
//                    }
//                })
//                .addData("roll", new Func<String>() {
//                    @Override public String value() {
//                        //return formatAngle(angles.angleUnit, angles.firstAngle);
//                        return Double.toString(robot.getRoll());
//                    }
//                });
//                .addData("glyph roll", new Func<String>() {
//                    @Override public String value() {
//                        //return formatAngle(angles.angleUnit, angles.firstAngle);
//                        return Double.toString(robot.glyphSystem.roll);
//                    }
//                })
//                .addData("glyph ticks", new Func<String>() {
//                    @Override public String value() {
//                        //return formatAngle(angles.angleUnit, angles.firstAngle);
//                        return Integer.toString(robot.glyphSystem.getMotorLiftPosition());
//                    }
//                });
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
//        telemetry.addLine()
//                .addData("auto stage", new Func<String>() {
//                    @Override public String value() {
//                        return String.valueOf(autoStage);
//                    }
//                })
                //.addData("glyph distance", new Func<String>() {
                //    @Override public String value() {
                //        return String.valueOf(robot.glyphUpper.getDistance(DistanceUnit.CM));
                //    }
                //})
//                .addData("TicksFL", new Func<String>() {
//                    @Override public String value() {
//                        return Long.toString(robot.motorFront.getElbowCurrentPos());
//                    }
//                })
//                .addData("TicksBL", new Func<String>() {
//                    @Override public String value() {
//                        return Long.toString(robot.motorBack.getElbowCurrentPos());
//                    }
//                })
//                .addData("TicksAvg", new Func<String>() {
//                    @Override public String value() {
//                        return Long.toString(robot.getAverageTicks());
//                    }
//                });
//        telemetry.addLine()
//
//                .addData("PID Calc", new Func<String>() {
//                    @Override public String value() {
//                        return Double.toString(robot.drivePID.performPID() );
//                    }
//                })
//                .addData("PID Err", new Func<String>() {
//                    @Override public String value() {
//                        return Double.toString(robot.drivePID.getError());
//                    }
//                });
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
