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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOp_6832", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class TeleOp_6832 extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    //Pele test motors

    DcMotor motorFrontLeft = null;
    DcMotor motorFrontRight = null;
    DcMotor motorBackLeft = null;
    DcMotor motorBackRight = null;
    DcMotor motorConveyor = null;
    DcMotor motorFlinger = null;
    private double powerFrontLeft = 0;
    private double powerFrontRight = 0;
    private double powerBackLeft = 0;
    private double powerBackRight = 0;
    private double powerConveyor = 0;
    private scoringSystem kobe = null;
    private long flingTimer = 0;
    private int flingSpeed = 5000; //ticks per second
    static final private long toggleLockout = (long)3e8; // fractional second lockout between all toggle button
    private long toggleOKTime = 0; //when should next toggle be allowed

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        this.motorFrontLeft = this.hardwareMap.dcMotor.get("motorFrontLeft");
        this.motorFrontRight = this.hardwareMap.dcMotor.get("motorFrontRight");
        this.motorBackLeft = this.hardwareMap.dcMotor.get("motorBackLeft");
        this.motorBackRight = this.hardwareMap.dcMotor.get("motorBackRight");
        this.motorConveyor = this.hardwareMap.dcMotor.get("motorConveyor");
        this.motorFlinger = this.hardwareMap.dcMotor.get("motorFlinger");


        this.motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motorConveyor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motorFlinger.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motorFlinger.setDirection(DcMotorSimple.Direction.REVERSE);

        this.kobe = new scoringSystem(flingSpeed, motorFlinger, motorConveyor);

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        // leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        // rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();



        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Status", "Left Joystick Y: " + Float.toString(gamepad1.left_stick_y));
            telemetry.addData("Status", "Right Joystick X: " + Float.toString(gamepad1.right_stick_x));
            telemetry.update();
            joystickDrive();

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
    public void joystickDrive(){

        /*powerBackRight = 0;
        powerFrontRight = 0;
        powerBackLeft = 0;
        powerFrontLeft = 0;

        if(Math.abs(gamepad1.left_stick_y) >= .10) {
            powerFrontLeft = gamepad1.left_stick_y;
            powerBackLeft = gamepad1.left_stick_y;
            powerFrontRight = gamepad1.left_stick_y;
            powerBackRight = gamepad1.left_stick_y;
        }
        if(Math.abs(gamepad1.left_stick_x) >= .10){
            powerFrontLeft += -gamepad1.left_stick_x;
            powerFrontRight += gamepad1.left_stick_x;
            powerBackLeft += gamepad1.left_stick_x;
            powerBackRight += -gamepad1.left_stick_x;
        }


        if(Math.abs(gamepad1.right_stick_x) >= .10){
            powerFrontLeft -= gamepad1.right_stick_x;
            powerBackLeft -= gamepad1.right_stick_x;
            powerFrontRight += gamepad1.right_stick_x;
            powerBackRight += gamepad1.right_stick_x;
        }

        */
        driveMixer(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);

        motorFrontLeft.setPower(powerFrontLeft);
        motorBackLeft.setPower(powerBackLeft);
        motorFrontRight.setPower(powerFrontRight);
        motorBackRight.setPower(powerBackRight);

        //toggle the particle conveyor on and off - quick and dirty
        if (gamepad1.a)
        {
            if(toggleAllowed())
            {
                kobe.collect();
            }
        }
        if (gamepad1.b)
        {
            if(toggleAllowed())
            {
                kobe.eject();
            }
        }
        if(toggleAllowed()) {
            if (gamepad1.x) {

                kobe.fling();
            }
        }
        if(gamepad1.y){
            if(toggleAllowed()) {
                if (kobe.isStopped()) {
                    kobe.restart();
                } else {
                    kobe.emergencyStop();
                }
            }
        }

        kobe.updateCollection();
    }

    public double clampMotor(double power) { return clampDouble(-1, 1, power); }

    public double clampDouble(double min, double max, double value)
    {
        double result = value;
        if(value > max)
            result = max;
        if(value < min)
            result = min;
        return result;
    }

    boolean toggleAllowed()
    {
        if (System.nanoTime()> toggleOKTime)
        {
            toggleOKTime= System.nanoTime()+toggleLockout;
            return true;
        }
        else
            return false;
    }
    public void driveMixer(double forward,double crab ,double rotate){
        powerBackRight = 0;
        powerFrontRight = 0;
        powerBackLeft = 0;
        powerFrontLeft = 0;


        powerFrontLeft = forward;
        powerBackLeft = forward;
        powerFrontRight = forward;
        powerBackRight = forward;


        powerFrontLeft += -crab;
        powerFrontRight += crab;
        powerBackLeft += crab;
        powerBackRight += -crab;


        powerFrontLeft -= rotate;
        powerBackLeft -= rotate;
        powerFrontRight += rotate;
        powerBackRight += rotate;


        clampMotor(powerFrontLeft);
        clampMotor(powerBackLeft);
        clampMotor(powerFrontRight);
        clampMotor(powerBackRight);

    }
}
