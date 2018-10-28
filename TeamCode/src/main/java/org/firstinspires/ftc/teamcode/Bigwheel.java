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

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.*;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.*;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Big Wheel", group="Linear Opmode")
public class Bigwheel extends LinearOpMode {

    private static final double LIFT_POWER = 0.5;
    private static final double LIFT_DELAY_MS = 400;
    private static final double LIFT_ENCODER_CHANGE = 1;
    private static final double LIFT_MIN = 0;
    private static final double LIFT_MAX = 1000;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor liftMotor = null;
    private DcMotor intakeMotor = null;
    //private DcMotor superman = null;

    private boolean isBlue = false;

    //boolean[] toggleState = new boolean[3];

    double[] liftPositions = {0, 400};
    double setEncoderLift = liftPositions[0];
    long lastLiftTimestamp = System.currentTimeMillis();

    //values associated with the buttons in the toggleAllowed method
    private boolean[] buttonSavedStates = new boolean[11];
    private int a = 0; //lower glyph lift
    private int b = 1; //toggle grip/release on glyph
    private int x = 2; //no function
    private int y = 3; //raise glyph lift
    private int dpad_down = 4; //glyph lift bottom position
    private int dpad_up = 5; //glyph lift top position
    private int dpad_left = 6; //no function
    private int dpad_right = 7; //glyph lift mid position
    private int left_bumper = 8; //increment state down (always)
    private int right_bumper = 9; //increment state up (always)
    private int startBtn = 10; //toggle active (always)

    int state = 0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        liftMotor = hardwareMap.get(DcMotor.class, "lift");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        //superman = hardwareMap.get(DcMotor.class, "superman");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(REVERSE);
        liftMotor.setMode(RUN_TO_POSITION);
        rightDrive.setDirection(FORWARD);
        intakeMotor.setDirection(FORWARD);
        //superman.setMode(RUN_TO_POSITION);

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
            if(toggleAllowed(gamepad1.dpad_left, dpad_left)){
                setEncoderLift = 0;
            }
            telemetry.addData("Status", "Initialized");
            telemetry.addData("Status", "Side: " + (isBlue ? "Blue" : "Red"));
            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }

        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.update();
            stateSwitch();
            if(opModeIsActive()) {
                switch(state){
                    case 0: //code for tele-op control
                        joystickDrive();
                        break;
                    case 1:
                        break;
                    case 2:
                        break;
                    case 3:
                        break;
                    case 4:
                        break;
                    case 5:
                        break;
                    case 6:
                        break;
                    case 7:
                        break;
                    case 8:
                        break;
                    case 9:
                        break;
                    case 10:
                        break;
                    default:
                        break;
                }
            }
            else {
                liftMotor.setPower(0);
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                intakeMotor.setPower(0);
            }
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }

    void joystickDrive() {
        double leftPower;
        double rightPower;

        double drive = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        leftPower = Range.clip(drive + turn, -1.0, 1.0);
        rightPower = Range.clip(drive - turn, -1.0, 1.0);
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);

        if(toggleAllowed(gamepad1.y, y)){

        }
        if(gamepad1.y){
            liftMotor.setPower(.75);
        }
        if(gamepad1.a){
            liftMotor.setPower(.75);
        }
        if(!gamepad1.y && !gamepad1.a){
            liftMotor.setPower(0);
        }
        if(gamepad1.x){
            intakeMotor.setPower(.65);
        }
        if(gamepad1.b){
            intakeMotor.setPower(-.65);
        }
        if(!gamepad1.x && !gamepad1.b){
            intakeMotor.setPower(0);
        }
        /*if(){
            intakeMotor.setPower(0);
        }*/
        /*if (gamepad1.dpad_up){
            superman.setPower(.75);
        }else if(gamepad1.dpad_down){
            superman.setPower(-.75);
        }else{
            superman.setPower(0);
        }*/


        //
        // liftMotor.setTargetPosition((int) setEncoderLift);

        /*if ((gamepad1.dpad_up || gamepad1.dpad_down) && System.currentTimeMillis() > lastLiftTimestamp + LIFT_DELAY_MS) {
            lastLiftTimestamp = System.currentTimeMillis();
            if (gamepad1.dpad_up)
                setEncoderLift = Range.clip(setEncoderLift + LIFT_ENCODER_CHANGE, LIFT_MIN, LIFT_MAX);
            else
                setEncoderLift = Range.clip(setEncoderLift - LIFT_ENCODER_CHANGE, LIFT_MIN, LIFT_MAX);
        }*/
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
       // telemetry.addData("Motors", "lift (%.2f), superman (%.2f)", liftMotor, superman);
        telemetry.update();
    }
    public void stateSwitch() {
        if(!opModeIsActive()) {
            if (toggleAllowed(gamepad1.left_bumper, left_bumper)) {
                state--;
                if (state < 0) {
                    state = 10;
                }
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                liftMotor.setPower(0);
                intakeMotor.setPower(0);
            }
            if (toggleAllowed(gamepad1.right_bumper, right_bumper)) {
                state++;
                if (state > 10) {
                    state = 0;
                }
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                liftMotor.setPower(0);
                intakeMotor.setPower(0);
            }
        }
        if (toggleAllowed(gamepad1.start, startBtn)) {
            leftDrive.setPower(0);
            rightDrive.setPower(0);
            liftMotor.setPower(0);
            intakeMotor.setPower(0);
        }
    }

    boolean toggleAllowed(boolean button, int buttonIndex) {
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
}
