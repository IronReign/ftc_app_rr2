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

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robots.handprosthetic.robopoglo;
import org.firstinspires.ftc.teamcode.statemachine.MineralStateProvider;
import org.firstinspires.ftc.teamcode.statemachine.Stage;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;
import org.firstinspires.ftc.teamcode.vision.Viewpoint;
import org.firstinspires.ftc.teamcode.vision.GoldPos;
import org.firstinspires.ftc.teamcode.vision.VisionProvider;
import org.firstinspires.ftc.teamcode.vision.VisionProviders;

import static org.firstinspires.ftc.teamcode.PoseBigWheel.servoNormalize;

/**
 * This file contains the code for Iron Reign's main OpMode, used for both TeleOp and Autonomous.
 */

@TeleOp(name = "Game_6832", group = "Challenge")  // @Autonomous(...) is the other common choice
//  @Autonomous
public class Game_6832 extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    private PoseBigWheel.RobotType currentBot;

    private PoseBigWheel robot;

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
    private long autoTimer = 0;
    private long autoDelay = 0;
    private Stage autoStage = new Stage();
    private Stage autoSetupStage = new Stage();

    //sensors/sensing-related variables
    private Orientation angles;

    //these are meant as short term testing variables, don't expect their usage
    //to be consistent across development sessions
    //private double testableDouble = robot.kpDrive;
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

    int stateLatched = -1;
    int stateIntake = -1;
    int stateDelatch = -1;
    boolean isIntakeClosed = true;
    boolean isHooked = false;
    boolean enableHookSensors = false;

    //game mode configuration
    private int gameMode = 0;
    private static final int NUM_MODES = 4;
    private static final String[] GAME_MODES = {"REGULAR", "ENDGAME", "PRE-GAME", "REVERE-REGULAR"};

    //vision-related configuration
    private VisionProvider vp;
    private int visionProviderState;
    private boolean visionProviderFinalized;
    private boolean enableTelemetry = false;
    private static final Class<? extends VisionProvider>[] visionProviders = VisionProviders.visionProviders;
    private static final Viewpoint viewpoint = Viewpoint.WEBCAM;
    private GoldPos initGoldPosTest = null;
    private int mineralState = 0;
    private MineralStateProvider mineralStateProvider = () -> mineralState;

    //sound related configuration
    private int soundState = 0;
    private int soundID = -1;

    //auto constants
    private static final double DRIVE_POWER = .95;
    private static final float TURN_TIME = 2;
    private static final float DUCKY_TIME = 0.5f;
    private double pCoeff = 0.26;
    private double dCoeff = 0.33;
    private double targetAngle = 273;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.log().add("Select robot -- (A) Icarus (Y)  Big Wheel");
        while (currentBot == null) {
            if (toggleAllowed(gamepad1.a, a)) {
                currentBot = PoseBigWheel.RobotType.Icarus;
                telemetry.log().add("Robot type: Icarus");
            }
            else if (toggleAllowed(gamepad1.y, y)) {
                currentBot = PoseBigWheel.RobotType.BigWheel;
                telemetry.log().add("Robot type:  Big Wheel");
            }
        }

        robot = new PoseBigWheel(currentBot);
        robot.init(this.hardwareMap, isBlue);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        configureDashboard();

        // waitForStart();
        // this is commented out but left here to document that we are still doing the
        // functions that waitForStart() normally does, but needed to customize it.

        robot.resetMotors(true);
        robot.collector.hookOn();
        robot.collector.closeGate();

        visionProviderFinalized = false;


        while (!isStarted()) {    // Wait for the game to start (driver presses PLAY)
            synchronized (this) {
                try {
                    this.wait();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                }
            }

            stateSwitch();


            //reset the elbow, lift and superman motors - operator must make sure robot is in the stowed position, flat on the ground
            if (toggleAllowed(gamepad1.b, b)) {
                robot.resetEncoders();
                robot.setZeroHeading();
                if (gamepad1.right_trigger > 0.3)
                    robot.setAutonomousIMUOffset(-45); //against field perimeter, facing left
                else
                    robot.setAutonomousIMUOffset(0); //against lander
                robot.collector.setElbowTargetPos(10, 1);
                robot.articulate(PoseBigWheel.Articulation.hanging);
                robot.collector.extendToMin();
            }

            if (toggleAllowed(gamepad1.x, x)) {
                isHooked = !isHooked;
                if (isHooked)
                    robot.collector.hookOff();
                else
                    robot.collector.hookOn();
            }

            if (toggleAllowed(gamepad1.y, y)) {
                autoDelay++;
                if (autoDelay > 20) autoDelay = 0;
            }

            /*if(toggleAllowed(gamepad1.a,a)){
                if(currentBot == PoseBigWheel.RobotType.Icarus){
                    currentBot = PoseBigWheel.RobotType.BigWheel;
                }else{
                    currentBot = PoseBigWheel.RobotType.Icarus;
                }
                stop();

            }*/

            if (toggleAllowed(gamepad1.left_stick_button, left_stick_button))
                enableHookSensors = !enableHookSensors;

            if (enableHookSensors && robot.distLeft.getDistance(DistanceUnit.METER) < .08)
                robot.collector.hookOn();
            if (enableHookSensors && robot.distRight.getDistance(DistanceUnit.METER) < .08)
                robot.collector.hookOff();

//            if(toggleAllowed(gamepad1.x,x)) {
//                isBlue = !isBlue;
//            }
//            if(toggleAllowed(gamepad1.a,a)){
//                autoDelay--;
//                if(autoDelay < 0) autoDelay = 20;
//            }


            if (!visionProviderFinalized && toggleAllowed(gamepad1.dpad_left, dpad_left)) {
                visionProviderState = (visionProviderState + 1) % visionProviders.length; //switch vision provider
            }
            if (!visionProviderFinalized && toggleAllowed(gamepad1.dpad_up, dpad_up)) {
                initialization_initVisionProvider(); //this is blocking
            } else if (visionProviderFinalized && toggleAllowed(gamepad1.dpad_up, dpad_up)) {
                initialization_deinitVisionProvider(); //also blocking, but should be very quick
            }
            if (!visionProviderFinalized && toggleAllowed(gamepad1.dpad_down, dpad_down)) {
                enableTelemetry = !enableTelemetry; //enable/disable FtcDashboard telemetry
            }
            if (visionProviderFinalized && gamepad1.left_trigger > 0.3) {
                GoldPos gp = vp.detect();
                if (gp != GoldPos.HOLD_STATE)
                    initGoldPosTest = gp;
                telemetry.addData("Vision", "Prep detection: %s%s", initGoldPosTest, gp == GoldPos.HOLD_STATE ? " (HOLD_STATE)" : "");
            }

            if (soundState == 0 && toggleAllowed(gamepad1.right_stick_button, right_stick_button)) {
                initialization_initSound();
            }

            telemetry.addData("Vision", "Backend: %s (%s)", visionProviders[visionProviderState].getSimpleName(), visionProviderFinalized ? "finalized" : System.currentTimeMillis() / 500 % 2 == 0 ? "**NOT FINALIZED**" : "  NOT FINALIZED  ");
            telemetry.addData("Vision", "FtcDashboard Telemetry: %s", enableTelemetry ? "Enabled" : "Disabled");
            telemetry.addData("Vision", "Viewpoint: %s", viewpoint);

            telemetry.addData("Sound", soundState == 0 ? "off" : soundState == 1 ? "on" : soundState == 2 ? "file not found" : "other");

            telemetry.addData("Status", "Initialized");
            telemetry.addData("Status", "Auto Delay: " + Long.toString(autoDelay) + "seconds");
            telemetry.addData("Status", "Side: " + getAlliance());
            telemetry.addData("Status", "Hook sensors: " + enableHookSensors);
            telemetry.update();

            robot.ledSystem.setColor(LEDSystem.Color.CALM);

            robot.updateSensors();

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }

        if (vp == null) {
            initialization_initDummyVisionProvider(); //this is blocking
        }

        vp.reset();

        robot.superman.restart(.75);
                robot.collector.restart(.4, .5);


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.update();
            stateSwitch();
            if (active) {
                switch (state) {
                    case 0: //code for tele-op control
                        joystickDrive();
                        break;
                    case 1: //autonomous that goes to opponent's crater
                        if (auto_depotSide.execute()) active = false;
                        break;
                    case 2: //autonomous that only samples
                        if (auto_depotSample.execute()) active = false;
                        break;
                    case 3: //autonomous that starts in our crater
                        if (auto_craterSide.execute()) active = false;
                        break;
                    case 4:
                        if (auto_driveStraight.execute()) active = false;
                        break;
                    case 5:
                        robot.setAutonSingleStep(false);
                        if (robot.getArticulation() == PoseBigWheel.Articulation.hanging)
                            robot.articulate(PoseBigWheel.Articulation.deploying); //start deploy sequence
                        break;
                    case 6:
                        if(driveStraight()) active = false;
                        break;
                    case 7:
//                        ledTest();
                        //servoTest();
                        /*
                        if (auto_turn.execute()) active = false;
                        telemetry.addData("Error: ", 90 - robot.getHeading());
                        telemetry.update();
                        */
                        if (toggleAllowed(gamepad1.y,y))
                            pCoeff+=.01;
                        else if (toggleAllowed(gamepad1.a, a))
                            pCoeff-=.01;
                        if (toggleAllowed(gamepad1.dpad_down, dpad_down))
                            dCoeff-=.005;
                        else if (toggleAllowed(gamepad1.dpad_up, dpad_down))
                            dCoeff+=.005;
                        if (toggleAllowed(gamepad1.right_bumper,right_bumper))
                            targetAngle+=.25;
                        else if (toggleAllowed(gamepad1.left_bumper, left_bumper))
                            targetAngle-=.25;
                        robot.balanceP = pCoeff;
                        robot.balanceD = dCoeff;
                        telemetry.addData("P Coeff: ", pCoeff);
                        telemetry.addData("D Coeff: ", dCoeff);
                        telemetry.addData("Target Ange: ", targetAngle);
                        telemetry.update();
                        robot.balance(targetAngle);



                        break;
                    case 8: //turn to IMU
                        robot.setAutonSingleStep(true);
                        demo();
                        break;
                    case 9:
                        if (auto_craterSide_extend_reverse.execute()) active = false;
                            break;
                    case 10:
                        if (auto_craterSide_extend.execute()) active = false;
                        break;

                    default:
                        robot.stopAll();
                        break;
                }
                robot.updateSensors();
            } else {
                robot.stopAll();
            }

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }

    public boolean driveStraight(){
        return robot.driveIMUDistance(0.01, 1, 0, false, 3.0);
    }

    private void initialization_initSound() {
        telemetry.addData("Please wait", "Initializing Sound");
        telemetry.update();
        robot.ledSystem.setColor(LEDSystem.Color.STRESS);
        soundID = hardwareMap.appContext.getResources().getIdentifier("gracious", "raw", hardwareMap.appContext.getPackageName());
        boolean success = SoundPlayer.getInstance().preload(hardwareMap.appContext, soundID);
        if (success)
            soundState = 1;
        else
            soundState = 2;
    }

    private void initialization_deinitVisionProvider() {
        telemetry.addData("Please wait", "Deinitializing vision");
        telemetry.update();
        robot.ledSystem.setColor(LEDSystem.Color.STRESS);
        vp.shutdownVision();
        vp = null;
        visionProviderFinalized = false;
    }

    private void initialization_initVisionProvider() {
        try {
            telemetry.addData("Please wait", "Initializing vision");
            telemetry.update();
            robot.ledSystem.setColor(LEDSystem.Color.STRESS);
            vp = visionProviders[visionProviderState].newInstance();
            vp.initializeVision(hardwareMap, telemetry, enableTelemetry, viewpoint);
        } catch (IllegalAccessException | InstantiationException e) {
            throw new RuntimeException(e);
        }
        visionProviderFinalized = true;
    }

    private void initialization_initDummyVisionProvider() {
        try {
            telemetry.addData("Please wait", "Initializing vision");
            telemetry.update();
            robot.ledSystem.setColor(LEDSystem.Color.STRESS);
            vp = VisionProviders.defaultProvider.newInstance();
            vp.initializeVision(hardwareMap, telemetry, enableTelemetry, viewpoint);
        } catch (IllegalAccessException | InstantiationException e) {
            throw new RuntimeException(e);
        }
        visionProviderFinalized = true;
    }

    private void demo() {
        if (gamepad1.x)
            robot.maintainHeading(gamepad1.x);

        if (gamepad1.dpad_down) {
            robot.articulate(PoseBigWheel.Articulation.manual);
            robot.superman.lower();
        }
        if (gamepad1.dpad_up) {
            robot.articulate(PoseBigWheel.Articulation.manual);
            robot.superman.raise();
        }
        if (gamepad1.dpad_right) {
            robot.articulate(PoseBigWheel.Articulation.manual);
            robot.collector.increaseElbowAngle();
        }
        if (gamepad1.dpad_left) {
            robot.articulate(PoseBigWheel.Articulation.manual);
            robot.collector.retractBelt();
        }

    }

    private StateMachine auto_turn = getStateMachine(autoStage)
            .addState(() -> robot.rotatePIDIMU(90, 3))
            .addTimedState(3, () -> {}, () -> {})
            .addState(() -> resetIMUBool())
            .build();

    private StateMachine auto_setup = getStateMachine(autoSetupStage)
            .addTimedState(autoDelay, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            .addSingleState(() -> robot.setAutonSingleStep(false)) //turn off autonSingleState
            //.addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.RED)) //red color
            .addSingleState(() -> robot.articulate(PoseBigWheel.Articulation.deploying)) //start deploy
            /*.addState(() -> {
                auto_sample();
                return  robot.getArticulation() == PoseBigWheel.Articulation.driving);
            })*/
            .addState(() -> robot.getArticulation() == PoseBigWheel.Articulation.driving) //wait until done
            .addState(() -> robot.articulate(PoseBigWheel.Articulation.driving, true))
            //.addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.PURPLE)) //purple color
            .addState(() -> robot.rotatePIDIMU(0, 1)) //turn back to center
            //.addTimedState(0.5f, () -> {}, () -> {}) //wait for the robot to settle down
            .addState(() -> robot.driveForward(false, .05, DRIVE_POWER)) //move back to see everything
            //.addTimedState(0.5f, () -> {}, () -> {}) //wait for the robot to settle down
            .addState(() -> auto_sample()) //detect the mineral
            .addState(() -> robot.driveForward(true, .05, DRIVE_POWER)) //move forward again
            .build();

    private StateMachine auto_setupReverse = getStateMachine(autoSetupStage)
            .addTimedState(autoDelay, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            .addSingleState(() -> robot.setAutonSingleStep(false)) //turn off autonSingleState
            //.addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.RED)) //red color
            .addSingleState(() -> robot.articulate(PoseBigWheel.Articulation.reversedeploying)) //start deploy
            .addState(() -> robot.getArticulation() == PoseBigWheel.Articulation.reverseDriving) //wait until done
            .addState(() -> robot.articulate(PoseBigWheel.Articulation.reverseDriving, true))
            //.addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.PURPLE)) //purple color
            .addState(() -> robot.rotatePIDIMU(0, 1)) //turn back to center
            //.addTimedState(0.5f, () -> {}, () -> {}) //wait for the robot to settle down
            //.addState(() -> robot.driveForward(false, .05, DRIVE_POWER)) //move back to see everything
            //.addTimedState(0.5f, () -> {}, () -> {}) //wait for the robot to settle down
            .addState(() -> auto_sample()) //detect the mineral
            //.addState(() -> robot.driveForward(true, .05, DRIVE_POWER)) //move forward again
            .build();

    private StateMachine auto_craterSide_extend_reverse = getStateMachine(autoStage)
            .addNestedStateMachine(auto_setupReverse)
            .addMineralState(mineralStateProvider, //turn to mineral
                    () -> robot.rotatePIDIMU(39+180, TURN_TIME),
                    () -> true,
                    () -> robot.rotatePIDIMU(321+180, TURN_TIME))

            .addMineralState(mineralStateProvider, //move to mineral
                    () -> robot.driveForward(true, .580, DRIVE_POWER),
                    () -> robot.driveForward(true, .50, DRIVE_POWER),
                    () -> robot.driveForward(true, .590, DRIVE_POWER))
            .addTimedState(.5f, () -> {}, () -> {})
            .addMineralState(mineralStateProvider, //move back
                    () -> robot.driveForward(false, .440, DRIVE_POWER),
                    () -> robot.driveForward(false, .45, DRIVE_POWER),
                    () -> robot.driveForward(false, .445, DRIVE_POWER))
            .addState(() -> robot.rotatePIDIMU(90, 3)) //turn parallel to minerals
            .addMineralState(mineralStateProvider, //move to wall
                    () -> robot.driveForward(true, 0.9, DRIVE_POWER),
                    () -> robot.driveForward(true, 1.1, DRIVE_POWER),
                    () -> robot.driveForward(true, 1.3, DRIVE_POWER))
            .addState(() -> robot.rotatePIDIMU(130, 3)) //turn to depot
            .addState(() -> robot.articulate(PoseBigWheel.Articulation.reverseDriving, true))
            .addState(() -> robot.articulate(PoseBigWheel.Articulation.manual, true))
            .addState(() -> robot.collector.setElbowTargetPos(10,1))
            .addState(() -> robot.driveForward(true, .5, DRIVE_POWER))
            .addState(() -> robot.collector.extendToMax(1,10))
            .addTimedState(DUCKY_TIME, //yeet ducky
                    () -> robot.collector.eject(),
                    () -> robot.collector.stopIntake())
            .addState(() -> robot.collector.extendToMid(1,10))
            .addState(() -> robot.articulate(PoseBigWheel.Articulation.reverseDriving, true))
            .addState(() -> robot.driveForward(false, .5, DRIVE_POWER))
            .addState(() -> robot.collector.nearTargetElbow())
            .addState(() -> robot.rotatePIDIMU(222, 0.6))
            .addState(() -> robot.rotatePIDIMU(315, 3))
            .addState(() -> robot.driveForward(true, .5, DRIVE_POWER))
            //.addState(() -> robot.articulate(PoseBigWheel.Articulation.preIntake, true))
            //.addState(() -> robot.articulate(PoseBigWheel.Articulation.manual, true))
            .build();


    private StateMachine auto_depotSide = getStateMachine(autoStage)
            .addNestedStateMachine(auto_setup)
            .addMineralState(mineralStateProvider, //turn to mineral
                    () -> robot.rotateIMU(39, TURN_TIME),
                    () -> true,
                    () -> robot.rotateIMU(321, TURN_TIME))
            .addMineralState(mineralStateProvider, //move to mineral
                    () -> robot.driveForward(true, .604, DRIVE_POWER),
                    () -> robot.driveForward(true, .47, DRIVE_POWER),
                    () -> robot.driveForward(true, .604, DRIVE_POWER))
            .addMineralState(mineralStateProvider, //turn to depot
                    () -> robot.rotateIMU(345, TURN_TIME),
                    () -> true,
                    () -> robot.rotateIMU(15, TURN_TIME))
            .addMineralState(mineralStateProvider, //move to depot
                    () -> robot.driveForward(true, .880, DRIVE_POWER),
                    () -> robot.driveForward(true, .762, DRIVE_POWER),
                    () -> robot.driveForward(true, .890, DRIVE_POWER))
            .addState(() -> robot.articulate(PoseBigWheel.Articulation.manual, true)) //so we can start overriding
            .addState(() -> robot.collector.setElbowTargetPos(618, 1))
            .addState(() -> robot.collector.extendToMid(1, 15))
            .addTimedState(DUCKY_TIME, //yeet ducky
                    () -> robot.collector.eject(),
                    () -> robot.collector.stopIntake())
            .addMineralState(mineralStateProvider, //turn to wall
                    () -> true,
                    () -> robot.rotateIMU(225, 4),
                    () -> robot.rotateIMU(225, 4))
            .addMineralState(mineralStateProvider, //move forward a little
                    () -> true,
                    () -> robot.driveForward(false, .090, DRIVE_POWER),
                    () -> robot.driveForward(false, .160, DRIVE_POWER))
            .addState(() -> robot.rotateIMU(303, 5)) //turn to crater
            .addState(() -> robot.driveForward(false, 1.05, DRIVE_POWER)) //go to crater
            .addState(() -> robot.rotateIMU(310, 1.5)) //turn to crater
            .addState(() -> robot.driveForward(false, .80, DRIVE_POWER)) //go to grater
            .addSingleState(() -> robot.collector.setElbowTargetPos(robot.collector.pos_AutoPark)) //extendBelt elbow to park
            .addState(() -> Math.abs(robot.collector.getElbowCurrentPos() - robot.collector.pos_AutoPark) < 20) //wait until done
            .build();

    private StateMachine auto_depotSample = getStateMachine(autoStage)
            .addNestedStateMachine(auto_setup)
            .addMineralState(mineralStateProvider, //turn to mineral
                    () -> robot.rotateIMU(39, TURN_TIME),
                    () -> true,
                    () -> robot.rotateIMU(321, TURN_TIME))
            .addMineralState(mineralStateProvider, //move to mineral
                    () -> robot.driveForward(true, .604, DRIVE_POWER),
                    () -> robot.driveForward(true, .47, DRIVE_POWER),
                    () -> robot.driveForward(true, .604, DRIVE_POWER))
            .addMineralState(mineralStateProvider, //turn to depot
                    () -> robot.rotateIMU(345, TURN_TIME),
                    () -> true,
                    () -> robot.rotateIMU(15, TURN_TIME))
            .addMineralState(mineralStateProvider, //move to depot
                    () -> robot.driveForward(true, .880, DRIVE_POWER),
                    () -> robot.driveForward(true, .762, DRIVE_POWER),
                    () -> robot.driveForward(true, .890, DRIVE_POWER))
            .addState(() -> robot.articulate(PoseBigWheel.Articulation.manual, true)) //so we can start overriding
            .addState(() -> robot.collector.setElbowTargetPos(618, 1))
            .addState(() -> robot.collector.extendToMid(1, 15))
            .addTimedState(DUCKY_TIME, //yeet ducky
                    () -> robot.collector.eject(),
                    () -> robot.collector.stopIntake())
            .build();

    private StateMachine auto_craterSide_extend = getStateMachine(autoStage)
            .addNestedStateMachine(auto_setup)
            .addMineralState(mineralStateProvider, //turn to mineral
                    () -> robot.rotatePIDIMU(39, TURN_TIME),
                    () -> true,
                    () -> robot.rotatePIDIMU(321, TURN_TIME))
            .addMineralState(mineralStateProvider, //move to mineral
                    () -> robot.driveForward(true, .880, DRIVE_POWER),
                    () -> robot.driveForward(true, .70, DRIVE_POWER),
                    () -> robot.driveForward(true, .890, DRIVE_POWER))
            .addMineralState(mineralStateProvider, //move back
                    () -> robot.driveForward(false, .440, DRIVE_POWER),
                    () -> robot.driveForward(false, .35, DRIVE_POWER),
                    () -> robot.driveForward(false, .445, DRIVE_POWER))
            .addState(() -> robot.rotatePIDIMU(270, 3)) //turn parallel to minerals
            .addMineralState(mineralStateProvider, //move to wall
                    () -> robot.driveForward(false, 1.43344, DRIVE_POWER),
                    () -> robot.driveForward(false, 1.48988, DRIVE_POWER),
                    () -> robot.driveForward(false, 1.9, DRIVE_POWER))
            .addState(() -> robot.rotatePIDIMU(310, 3)) //turn to depot
            .addState(() -> robot.articulate(PoseBigWheel.Articulation.preIntake, true))
            .addState(() -> robot.articulate(PoseBigWheel.Articulation.manual, true))
            .addState(() -> robot.collector.extendToMax(1,10))
            .addTimedState(DUCKY_TIME, //yeet ducky
                    () -> robot.collector.eject(),
                    () -> robot.collector.stopIntake())
            .addState(() -> robot.collector.extendToMid(1,10))
            .addState(() -> robot.articulate(PoseBigWheel.Articulation.driving, true))
            .addState(() -> robot.collector.nearTargetElbow())
            .addState(() -> robot.rotatePIDIMU(100, 0.6))
            .addState(() -> robot.rotatePIDIMU(130, 3))
            .addState(() -> robot.driveForward(false, .5, DRIVE_POWER))
            .addState(() -> robot.articulate(PoseBigWheel.Articulation.preIntake, true))
            .addState(() -> robot.articulate(PoseBigWheel.Articulation.manual, true))


            /*.addState(() -> robot.driveForward(true, 1.2, DRIVE_POWER)) //move to depot
            .addState(() -> robot.articulate(PoseBigWheel.Articulation.manual, true)) //so we can start overriding
            .addState(() -> robot.collector.setElbowTargetPos(618, 1))
            .addState(() -> robot.collector.extendToMid(1, 15))
            .addTimedState(DUCKY_TIME, //yeet ducky
                    () -> robot.collector.eject(),
                    () -> robot.collector.stopIntake())
            .addState(() -> robot.driveForward(false, 2, DRIVE_POWER))
            .addSingleState(() -> robot.collector.setElbowTargetPos(robot.collector.pos_AutoPark)) //extendBelt elbow to park
            .addState(() -> Math.abs(robot.collector.getElbowCurrentPos() - robot.collector.pos_AutoPark) < 20) //wait until done*/
            .build();


    private boolean resetIMUBool() {
        robot.resetIMU();
        return true;
    }
    private StateMachine auto_craterSide = getStateMachine(autoStage)
            .addNestedStateMachine(auto_setup)
            .addMineralState(mineralStateProvider, //turn to mineral
                    () -> robot.rotateIMU(39, TURN_TIME),
                    () -> true,
                    () -> robot.rotateIMU(321, TURN_TIME))
            .addMineralState(mineralStateProvider, //move to mineral
                    () -> robot.driveForward(true, .880, DRIVE_POWER),
                    () -> robot.driveForward(true, .70, DRIVE_POWER),
                    () -> robot.driveForward(true, .890, DRIVE_POWER))
            .addMineralState(mineralStateProvider, //move back
                    () -> robot.driveForward(false, .440, DRIVE_POWER),
                    () -> robot.driveForward(false, .35, DRIVE_POWER),
                    () -> robot.driveForward(false, .445, DRIVE_POWER))
            .addState(() -> robot.rotateIMU(80, TURN_TIME)) //turn parallel to minerals
            .addMineralState(mineralStateProvider, //move to wall
                    () -> robot.driveForward(true, 1.4, DRIVE_POWER),
                    () -> robot.driveForward(true, 1.5, DRIVE_POWER),
                    () -> robot.driveForward(true, 1.9, DRIVE_POWER))
            .addState(() -> robot.rotateIMU(135, TURN_TIME)) //turn to depot
            .addState(() -> robot.driveForward(true, 1.2, DRIVE_POWER)) //move to depot
            .addState(() -> robot.articulate(PoseBigWheel.Articulation.manual, true)) //so we can start overriding
            .addState(() -> robot.collector.setElbowTargetPos(618, 1))
            .addState(() -> robot.collector.extendToMid(1, 15))
            .addTimedState(DUCKY_TIME, //yeet ducky
                    () -> robot.collector.eject(),
                    () -> robot.collector.stopIntake())
            .addState(() -> robot.driveForward(false, 2, DRIVE_POWER))
            .addSingleState(() -> robot.collector.setElbowTargetPos(robot.collector.pos_AutoPark)) //extendBelt elbow to park
            .addState(() -> Math.abs(robot.collector.getElbowCurrentPos() - robot.collector.pos_AutoPark) < 20) //wait until done
            .build();

    private StateMachine auto_driveStraight = getStateMachine(autoStage)
            .addState(() -> robot.driveForward(false, 4, DRIVE_POWER))
            .build();




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


    private void joystickDrive() {

        if (!joystickDriveStarted) {
            robot.resetMotors(true);
            robot.setAutonSingleStep(true);
            isHooked = false;
            joystickDriveStarted = true;
        }

        pwrFwd = direction * pwrDamper * gamepad1.left_stick_y;
        pwrStf = direction * pwrDamper * gamepad1.left_stick_x;
        pwrRot = -pwrDamper * .75 * gamepad1.right_stick_x;


        pwrFwdL = direction * pwrDamper * gamepad1.left_stick_y;
        pwrStfL = direction * pwrDamper * gamepad1.left_stick_x;

        pwrFwdR = direction * pwrDamper * gamepad1.right_stick_y;
        pwrStfR = direction * pwrDamper * gamepad1.right_stick_x;

       /* if ((robot.getRoll() > 300) && robot.getRoll() < 350)
            //todo - needs improvement - should be enabling slowmo mode, not setting the damper directly
            //at least we are looking at the correct axis now - it was super janky - toggling the damper as the axis fluttered across 0 to 365
            pwrDamper = .33;
        else*/
            pwrDamper = 1.0;


        switch (gameMode) {
            case 0: //regular teleop mineral cycle
                joystickDriveRegularMode();
                break;
            case 1: //endgame mode
                joystickDriveEndgameMode();
                break;
            case 2: //pre-game = testing auton deploying functions
                joystickDrivePregameMode();
                break;
            case 3:
                joystickDriveRegularModeReverse();
                break;
        }

        //manual control
        if (gamepad1.dpad_down) {
            robot.articulate(PoseBigWheel.Articulation.manual);
            robot.superman.lower();
        }
        if (gamepad1.dpad_up) {
            robot.articulate(PoseBigWheel.Articulation.manual);
            robot.superman.raise();
        }

        if (gamepad1.right_stick_y > 0.5) {
            robot.articulate(PoseBigWheel.Articulation.manual);
            robot.collector.decreaseElbowAngle();
        }
        if (gamepad1.right_stick_y < -0.5) {
            robot.articulate(PoseBigWheel.Articulation.manual);
            robot.collector.extendBelt();
        }

        if (gamepad1.dpad_right) {
            robot.articulate(PoseBigWheel.Articulation.manual);
            robot.collector.increaseElbowAngle();
        }
        if (gamepad1.dpad_left) {
            robot.articulate(PoseBigWheel.Articulation.manual);
            robot.collector.retractBelt();
        }


        //elbow control
        currTarget = robot.collector.getExtendABobTargetPos();
        if (toggleAllowed(gamepad1.left_bumper, left_bumper)) {
            if (currTarget == robot.collector.extendMid) {
                currTarget = robot.collector.extendMax;
            } else {
                currTarget = robot.collector.extendMid;
            }

        }
        if (currTarget >= 10) {
            robot.collector.setExtendABobTargetPos(currTarget);
        }


        //endgame mode
        if (toggleAllowed(gamepad1.right_bumper, right_bumper)) {
            gameMode = (gameMode + 1) % NUM_MODES;
        }


        //intake code
        double triggers = gamepad1.left_trigger - gamepad1.right_trigger;
        if (triggers > 0.1)
            robot.collector.collect();
        else if (triggers < -0.1)
            robot.collector.eject();
        else
            robot.collector.stopIntake();

        //Gracious Professionalism!
        if (soundState == 1 && toggleAllowed(gamepad1.right_stick_button, right_stick_button)) {
            SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, soundID);
        }

    }

    private void joystickDrivePregameMode() {
        robot.setAutonSingleStep(true); //single step through articulations having to do with deploying

        robot.ledSystem.setColor(LEDSystem.Color.GOLD);

        boolean doDelatch = false;
        if (toggleAllowed(gamepad1.b, b)) {
            stateDelatch++;
            if (stateDelatch > 2) stateDelatch = 0;
            doDelatch = true;
        }

        if (toggleAllowed(gamepad1.x, x)) {
            stateDelatch--;
            if (stateDelatch < 0) stateDelatch = 2;
            doDelatch = true;
        }

        if (doDelatch) {
            switch (stateDelatch) {
                case 0:
                    robot.articulate(PoseBigWheel.Articulation.hanging);
                    break;
                case 1:
                    robot.articulate(PoseBigWheel.Articulation.deploying);
                    break;
                case 2:
                    robot.articulate(PoseBigWheel.Articulation.deployed);
                    break;
                default:
                    break;
            }
        }
    }

    private void logTurns(double target) {
        telemetry.addData("Error: ", target - robot.getHeading());
        telemetry.update();
    }

    private void joystickDriveEndgameMode() {

        robot.ledSystem.setColor(LEDSystem.Color.BLUE);

        boolean doLatchStage = false;
        robot.driveMixerTank(pwrFwd, pwrRot);
        if (toggleAllowed(gamepad1.b, b)) { //b advances us through latching stages - todo: we should really be calling a pose.nextLatchStage function
            stateLatched++;
            if (stateLatched > 2) stateLatched = 0;
            doLatchStage = true;
        }

        if (toggleAllowed(gamepad1.x, x)) { //x allows us to back out of latching stages
            stateLatched--;
            if (stateLatched < 0) stateLatched = 0;
            doLatchStage = true;
        }

        if (doLatchStage) {
            switch (stateLatched) {
                case 0:
                    robot.articulate(PoseBigWheel.Articulation.latchApproach);
                    break;
                case 1:
                    robot.articulate(PoseBigWheel.Articulation.latchPrep);
                    break;
                case 2:
                    robot.articulate(PoseBigWheel.Articulation.latchSet);
                    break;
            }
        }

        if (toggleAllowed(gamepad1.a, a)) {
            isHooked = !isHooked;
        }

        if (isHooked) {
            robot.collector.hookOn();
        } else {
            robot.collector.hookOff();
        }
    }

    private boolean turnTest(double angle, double maxTime) {
        return robot.rotatePIDIMU(angle, maxTime);
    }

    private void joystickDriveRegularMode() {

        robot.ledSystem.setColor(LEDSystem.Color.GAME_OVER);

        robot.collector.hookOff();

        boolean doIntake = false;
        robot.driveMixerTank(pwrFwd, pwrRot);

        if (gamepad1.y) {
            robot.goToSafeDrive();
            isIntakeClosed = true;
        }
        if (toggleAllowed(gamepad1.a, a)) {
            isIntakeClosed = !isIntakeClosed;
        }


        if (toggleAllowed(gamepad1.b, b)) {
            stateIntake++;
            if (stateIntake > 3) stateIntake = 0;
            doIntake = true;
        }

        if (toggleAllowed(gamepad1.x, x)) {
            stateIntake--;
            if (stateIntake < 0) stateIntake = 3;
            doIntake = true;
        }

        if (doIntake) {
            switch (stateIntake) {
                case 0:
                    robot.articulate(PoseBigWheel.Articulation.preIntake);
                    isIntakeClosed = true;
                    break;
                case 1:
                    robot.articulate(PoseBigWheel.Articulation.intake);
                    isIntakeClosed = true;
                    break;
                case 2:
                    robot.articulate(PoseBigWheel.Articulation.deposit);
                    break;
                case 3:
                    robot.articulate(PoseBigWheel.Articulation.driving);
                    isIntakeClosed = true;
            }
        }


        if (isIntakeClosed) {
            robot.collector.closeGate();
        } else {
            robot.collector.openGate();
        }
    }

    private void joystickDriveRegularModeReverse() {

        robot.ledSystem.setColor(LEDSystem.Color.GAME_OVER);

        robot.collector.hookOff();

        boolean doIntake = false;
        robot.driveMixerTank(pwrFwd, pwrRot);

        if (gamepad1.y) {
            robot.articulate(PoseBigWheel.Articulation.reverseDriving);
            isIntakeClosed = true;
        }
        if (toggleAllowed(gamepad1.a, a)) {
            isIntakeClosed = !isIntakeClosed;
        }


        if (toggleAllowed(gamepad1.b, b)) {
            stateIntake++;
            if (stateIntake > 3) stateIntake = 0;
            doIntake = true;
        }

        if (toggleAllowed(gamepad1.x, x)) {
            stateIntake--;
            if (stateIntake < 0) stateIntake = 3;
            doIntake = true;
        }

        if (doIntake) {
            switch (stateIntake) {
                case 0:
                    robot.articulate(PoseBigWheel.Articulation.reverseIntake);
                    isIntakeClosed = true;
                    break;
                case 1:
                    robot.articulate(PoseBigWheel.Articulation.prereversedeposit);
                    isIntakeClosed = true;
                    break;
                case 2:
                    robot.articulate(PoseBigWheel.Articulation.reverseDeposit);
                    break;
                case 3:
                    robot.articulate(PoseBigWheel.Articulation.reverseDriving);
                    isIntakeClosed = true;
                    break;
            }
        }


        if (isIntakeClosed) {
            robot.collector.closeGate();
        } else {
            robot.collector.openGate();
        }
    }


    //the method that controls the main state of the robot; must be called in the main loop outside of the main switch
    private void stateSwitch() {
        if (!active) {
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
            if (!buttonSavedStates[buttonIndex]) { //we just pushed the button, and when we last looked at it, it was not pressed
                buttonSavedStates[buttonIndex] = true;
                return true;
            } else { //the button is pressed, but it was last time too - so ignore

                return false;
            }
        }

        buttonSavedStates[buttonIndex] = false; //not pressed, so remember that it is not
        return false; //not pressed

    }


    private String getAlliance() {
        if (isBlue)
            return "Blue";
        return "Red";
    }


    private void configureDashboard() {
        // Configure the dashboard.

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(() ->
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = robot.imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX)

        );

        telemetry.addLine()
                .addData("active", () -> active)
                .addData("state", () -> state)
                .addData("autoStage", () -> autoStage);
        telemetry.addLine()
                .addData("elbowA", () -> robot.collector.isActive())
                .addData("elbowC", () -> robot.collector.getElbowCurrentPos())
                .addData("elbowC2", () -> robot.collector.getElbowCurrentPos2())
                .addData("elbowT", () -> robot.collector.getElbowTargetPos());
        telemetry.addLine()
                .addData("supermanPos", () -> robot.superman.getCurrentPosition())
                .addData("liftPos", () -> robot.collector.getExtendABobCurrentPos());
        telemetry.addLine()
                .addData("COG: ", () -> robot.cog.getCenterOfGravity(robot.getRoll() +robot.cog.pitchOffset,180-(robot.superman.getCurrentAngle() + robot.cog.supermanoffset), robot.collector.getCurrentAngle()+robot.cog.elbowoffset, .382).toString());
        telemetry.addLine()
                .addData("roll", () -> robot.getRoll())
                .addData("pitch", () -> robot.getPitch())
                .addData("yaw", () -> robot.getHeading());
        telemetry.addLine()
                .addData("calib", () -> robot.imu.getCalibrationStatus().toString());

        telemetry.addLine()
                .addData("status", () -> robot.imu.getSystemStatus().toShortString())
                .addData("Pos", () -> robot.intakeGate.getPosition())
                .addData("mineralState", () -> mineralState)
                .addData("Game Mode", () -> GAME_MODES[gameMode])
                .addData("distForward", () -> robot.distForward.getDistance(DistanceUnit.METER))
                .addData("distLeft", () -> robot.distLeft.getDistance(DistanceUnit.METER))
                .addData("distRight", () -> robot.distRight.getDistance(DistanceUnit.METER));
//        telemetry.addData("TOFID", () -> String.format("%x", robot.sensorTimeOfFlight.getModelID()));
//        telemetry.addData("TOF did time out", () -> Boolean.toString(robot.sensorTimeOfFlight.didTimeoutOccur()));
    }

    int testDeLatch = 0;
    public boolean delatch() {
        switch (testDeLatch) {
            case 0:
                robot.collector.setElbowTargetPos(robot.collector.pos_prelatch, .9);
                if (robot.collector.nearTargetElbow()) {
                    testDeLatch++;
                }
                break;
            case 1:
                robot.superman.setTargetPosition(robot.superman.pos_prelatch);
                if (robot.superman.nearTarget()) {
                    robot.collector.hookOff();
                    testDeLatch++;
                }
                break;
            default:
                testDeLatch = 0;
                return true;
        }
        return false;
    }

    private int servoTest = 900;

    private void servoTest() {
        robot.ledSystem.movement.setPosition(servoNormalize(servoTest));
        if (toggleAllowed(gamepad1.a, a))
            servoTest -= 20;
        else if (toggleAllowed(gamepad1.y, y))
            servoTest += 20;
        telemetry.addData("Pulse width", servoTest);
    }

    private void ledTest() {
        int idx = (int) ((System.currentTimeMillis() / 2000) % LEDSystem.Color.values().length);
        robot.ledSystem.setColor(LEDSystem.Color.values()[idx]);
        telemetry.addData("Color", LEDSystem.Color.values()[idx].name());
    }

    private StateMachine.Builder getStateMachine(Stage stage) {
        return StateMachine.builder()
                .stateSwitchAction(() -> robot.resetMotors(true))
                .stateEndAction(() -> {})
                .stage(stage);
    }

}
