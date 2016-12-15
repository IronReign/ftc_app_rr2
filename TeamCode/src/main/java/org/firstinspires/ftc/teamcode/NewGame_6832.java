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

import java.util.Locale;

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

@TeleOp(name="GnuGame_6832", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//  @Autonomous

public class NewGame_6832 extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    private Pose deadShot = new Pose();

    SoundPlayer deadShotSays = SoundPlayer.getInstance();




    private boolean active = true;


    static final private long toggleLockout = (long)2e8; // fractional second lockout between all toggle button
    private long toggleOKTime = 0; //when should next toggle be allowed
    private int autoState = 0;
    private int beaconState = 0;
    private boolean initiallized = false;
    private int flingNumber = 2;
    private boolean isBlue = false;
    private boolean targetBeacon = true;
    private double IMUTargetHeading = 0;
    Orientation angles;

    private long presserTimer = 0;
    private int state = 0;
    private boolean runAutonomous = true;
    private long autoTimer = 0;
    private boolean[] buttonSavedStates = new boolean[8];
    //private boolean[] buttonCurrentState = new boolean[8];
    private boolean slowMode = false;

    private int pressedPosition = 750; //Note: find servo position value for pressing position on pushButton
    private int relaxedPosition = 2250; //Note: find servo position value for relaxing position on pushButton

    @Override
    public void runOpMode() throws InterruptedException {

        deadShot.init(this.hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        configureDashboard();

        while(!isStarted()){
            synchronized (this) {
                try {
                    this.wait();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                }
            }

            stateSwitch();

            if(gamepad1.a){
                flingNumber = 1;
            }
            if(gamepad1.b){
                flingNumber = 2;
            }
            if(gamepad1.y){
                flingNumber = 3;
            }
            if(toggleAllowed(gamepad1.x,2)) {

                    isBlue = !isBlue;

            }
            if(toggleAllowed(gamepad1.dpad_down,4)){

                    deadShot.pa.halfCycle();

            }

            telemetry.addData("Status", "Initialized");
            telemetry.addData("Status", "Number of throws: " + Integer.toString(flingNumber));
            telemetry.addData("Status", "Side: " + getAlliance());
            telemetry.update();

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        // leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        // rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors




        // Wait for the game to start (driver presses PLAY)
//        waitForStart();
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
                    case 0:
                        autonomous();
                        break;
                    case 1:
                        joystickDrive();
                        break;
                    case 2:
                        deadShot.pa.halfCycle();
                        active = false;
                        break;
                    case 3:
                        secondaryAuto();
                        break;
                }
                deadShot.updateSensors();
            }

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
    public void autonomous(){
        if(autoState == 0)
            autoTimer = System.nanoTime() + (long) 30e9;
        if(autoTimer > System.nanoTime()) {
            switch (autoState) {
                case -1: //sit and spin - do nothing
                    break;

                case 0: //reset all the motors before starting autonomous
                    //autoTimer = System.nanoTime() + (long) 30e9;
                    deadShot.resetMotors();
                    autoState++;
                    deadShotSays.play(hardwareMap.appContext, R.raw.a01);
                    break;
                case 1: //drive forward and shoot in the goal

                    if (deadShot.driveForward(true, 1.45, 1)) {
                        deadShot.resetMotors();
                        for (int n = 0; n < flingNumber; n++)
                            deadShot.pa.fling();
                        autoState++;
                        deadShotSays.play(hardwareMap.appContext, R.raw.a02);
                    }
                    break;
                case 2:  // 180 degree turn if alternate alliance, then moves are inverted
                    if (isBlue) {

                        if (deadShot.rotateRelative(true, 90, .30)) {
                            deadShot.targetAngleInitialized = false;
                            deadShot.resetMotors();
                            autoState++;
                        }
                        else autoState++; deadShotSays.play(hardwareMap.appContext, R.raw.a03);
                    } else {
                        autoState++;
                        deadShotSays.play(hardwareMap.appContext, R.raw.a03);
//                        active = false;
                    }
                    break;
                case 3: //drive towards the corner vortex

                    if (deadShot.driveStrafe(true, 1, 1)) {
                        deadShot.resetMotors();
                        deadShotSays.play(hardwareMap.appContext, R.raw.a04);
                        autoState++;
                    }
                    break;
                case 4: //drive up near the first beacon

                    if (deadShot.driveForward(!isBlue, .20, 1)) {
                        deadShot.resetMotors();
                        autoState++;
                    }
                    break;
                case 5: //drive towards the wall in order to press the first beacon
                    if (deadShot.driveStrafe(true, .5, 1)) {
                        deadShot.resetMotors();
                        autoState++;
                    }
                    break;
                case 6: //press the first beacon
                    if (deadShot.pressTeamBeacon(isBlue)) {
                        deadShot.resetMotors();
                        autoState++;
                    }
                    break;
                case 7: //drive up next to the second beacon
                    if (deadShot.driveForward(!isBlue, 1.0, 1)) {
                        deadShot.resetMotors();
                        autoState++;
                    }
                    break;
                case 8: //press the second beacon
                    if (deadShot.pressTeamBeacon(isBlue)) {
                        deadShot.resetMotors();
                        autoState++;
                    }
                    break;
//            case 9: //drive away from the second beacon
//                if(driveCrab(false, 0, .5)) {
//                    resetMotors();
//                    autoState++;
//                }

                default:
                    break;
            }
            deadShot.pa.updateCollection();
        }
        else{
            deadShot.pa.emergencyStop();
            deadShot.driveMixer(0, 0, 0);
        }
    }

    public void stateSwitch(){
        /*button indexes:
        0 = a
        1 = b
        2 = x
        3 = y
        4 = dpad_down
        5 = left bumper
        6 = right bumper
        7 = start button
        */
//        buttonCurrentState[0] = gamepad1.a;
//        buttonCurrentState[1] = gamepad1.b;
//        buttonCurrentState[2] = gamepad1.x;
//        buttonCurrentState[3] = gamepad1.y;
//        buttonCurrentState[4] = gamepad1.dpad_down;
//        buttonCurrentState[5] = gamepad1.left_bumper;
//        buttonCurrentState[6] = gamepad1.right_bumper;
//        buttonCurrentState[7] = gamepad1.start;

        if(toggleAllowed(gamepad1.left_bumper,5)) {

                state--;
                if (state < 0) {
                    state = 3;
                }
                active = false;

        }

        if (toggleAllowed(gamepad1.right_bumper,6)) {

                state++;
                if (state > 3) {
                    state = 0;
                }
                active = false;

        }

        if(toggleAllowed(gamepad1.start,7)) {

                active = !active;

        }
    }

    public void secondaryAuto(){
        switch(autoState){
            case 0:
                deadShot.resetMotors();
                autoState++;
                break;
            case 1:
                if(deadShot.driveForward(true, 2.25, 1)){
                    deadShot.resetMotors();
                    autoState++;
                }
                break;
            case 2:
                for(int n = 0; n < flingNumber; n++){
                    deadShot.pa.fling();
                }
                autoState++;
                deadShot.motorConveyor.setPower(0);
                break;
            case 3:
                if(deadShot.driveForward(true, .5, 1)){
                    deadShot.resetMotors();
                    autoState++;
                }
            default:
                break;
        }
    }


    public void joystickDrive(){


        deadShot.driveMixer(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);

        //toggle the particle conveyor on and off - quick and dirty
        if (toggleAllowed(gamepad1.a,0))
        {
            deadShot.pa.collect();
        }

        if (toggleAllowed(gamepad1.b,1))
        {
                deadShot.pa.eject();

        }

        if(toggleAllowed(gamepad1.x,2)) {


                deadShot.pa.fling();

        }

        if(toggleAllowed(gamepad1.y,3)){

                if (deadShot.pa.isStopped()) {
                    deadShot.pa.restart();
                } else {
                    deadShot.pa.emergencyStop();
                }
        }

        if(toggleAllowed(gamepad1.dpad_down,4)){

                deadShot.pa.halfCycle();

        }

        deadShot.pa.updateCollection();
    }





    public boolean foundBeacon() {
        if(isBlue){
            return deadShot.beaconDistAft > 0.05;
        }
        return deadShot.beaconDistFore > .05;
    }

    public boolean findBeaconPressRange() {
        double dist;
        if(isBlue){ dist = deadShot.beaconDistAft; }
        else { dist = deadShot.beaconDistFore; }
        if(dist > .25){
            deadShot.driveMixer(0, -.35, 0);
            return false;
        }
        else if(dist < .15){
            deadShot.driveMixer(0, .35, 0);
            return false;
        }
        else{
            deadShot.driveMixer(0, 0, 0);
            return true;
        }
    }

    public boolean onTeamColor(){
        if(isBlue){
            return deadShot.colorAft == 3;
        }
        return (deadShot.colorFore > 9 && deadShot.colorFore < 12);
    }




    boolean toggleAllowed(boolean button, int buttonIndex)
    {
        /*button indexes:
        0 = a
        1 = b
        2 = x
        3 = y
        4 = dpad_down
        5 = left bumper
        6 = right bumper
        7 = start button
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

    public String autoRun(){
        if(runAutonomous)
            return "Auto will run";
        return "Auto will not run";
    }

    public double ServoNormalize(int pulse){
        double normalized = (double)pulse;
        return (normalized - 750.0) / 1500.0; //convert mr servo controller pulse width to double on 0 - 1 scale
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
                angles = deadShot.imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
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
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return deadShot.imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return deadShot.imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });
        telemetry.addLine()
                .addData("State", new Func<String>() {
                    @Override public String value() {
                        return String.valueOf(autoState);
                    }
                })
                .addData("TicksFL", new Func<String>() {
                    @Override public String value() {
                        return Long.toString(deadShot.motorFrontLeft.getCurrentPosition());
                    }
                })
                .addData("TicksAvg", new Func<String>() {
                    @Override public String value() {
                        return Long.toString(deadShot.getAverageTicks());
                    }
                });
        telemetry.addLine()
                .addData("DistRear", new Func<String>() {
                    @Override public String value() {
                        return String.valueOf(deadShot.beaconDistAft);
                    }
                })
                .addData("RearColor", new Func<String>() {
                    @Override public String value() {
                        return Long.toString(deadShot.colorAft);
                    }
                })
                .addData("DistFore", new Func<String>() {
                    @Override public String value() {
                        return String.valueOf(deadShot.beaconDistFore);
                    }
                })
                .addData("ForeColor", new Func<String>() {
                    @Override public String value() {
                        return Long.toString(deadShot.colorFore);
                    }
                });

//        telemetry.addData("Status", "Run Time: " + runtime.toString());
//        //telemetry.addData("Status", "State: " + autoState);
//        //telemetry.addData("Status", "Front Left Ticks: " + Long.toString(motorFrontLeft.getCurrentPosition()));
//        //telemetry.addData("Status", "Average Ticks: " + Long.toString(getAverageTicks()));
//        telemetry.addLine().addData("Normal", beaconPresentRear.getLightDetected());
//
//        telemetry.addLine().addData("ColorFore", colorForeCache[0] & 0xFF);
//        telemetry.addData("ColorRear", colorRearCache[0] & 0xFF);

    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
