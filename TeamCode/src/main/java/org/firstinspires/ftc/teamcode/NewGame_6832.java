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

    private Pose robot = new Pose();

    SoundPlayer deadShotSays = SoundPlayer.getInstance();




    private boolean active = true;
    boolean joystickDriveStarted = false;

    private int autoState = 0;
    private int beaconState = 0;

    private int flingNumber = 0;
    private boolean isBlue = false;

    Orientation angles;

    private int state = 0;
    private boolean runAutonomous = true;
    private long autoTimer = 0;
    private boolean[] buttonSavedStates = new boolean[11];
    //private boolean[] buttonCurrentState = new boolean[8];
    private boolean slowMode = false;

    private boolean runDemo = false;
    private boolean runBeaconTestLeft = true;


    private int pressedPosition = 750; //Note: find servo position value for pressing position on pushButton
    private int relaxedPosition = 2250; //Note: find servo position value for relaxing position on pushButton

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(this.hardwareMap);

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
                flingNumber = 0;
            }
            if(toggleAllowed(gamepad1.x,2)) {

                    isBlue = !isBlue;

            }
            if(toggleAllowed(gamepad1.dpad_down,4)){

                    robot.particle.halfCycle();

            }

            telemetry.addData("Status", "Initialized");
            telemetry.addData("Status", "Number of throws: " + Integer.toString(flingNumber));
            telemetry.addData("Status", "Side: " + getAlliance());
            telemetry.update();

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }


        // Wait for the game to start (driver presses PLAY)
        //        waitForStart(); //this is commented out but left here to document that we are still doing the functions that waitForStart() normally does, but needed to customize it.
        //todo However, it's clearly not happening at this location any more, so we need to move this comment to where it should be
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
                    case 0: //main autonomous function that scores 1 or 2 balls and toggles both beacons
                        joystickDriveStarted = false;
                        autonomous(1);
                        break;
                    case 1: //this is the autonomous we use if our teamates can also go for the beacons more reliably than we can; scores 2 balls and pushes the cap ball, also parks on the center element
                        joystickDriveStarted = false;
                        secondaryAuto();
                        break;
                    case 2: //code for tele-op control
                        joystickDrive();
                        break;
                    case 3: //half-cycles the flinger
                        joystickDriveStarted = false;
                        robot.particle.halfCycle();
                        active = false;
                        break;
                    case 4: //provides data for forwards/backwards calibration
                        joystickDriveStarted = false;
                        if(robot.getAverageTicks() < 2000){
                            robot.driveMixer(1,0,0);
                        }
                        else robot.driveMixer(0,0,0);
                        break;
                    case 5: //provides data for left/right calibration
                        joystickDriveStarted = false;
                        if(robot.getAverageAbsTicks() < 2000){
                            robot.driveMixer(0,1,0);
                        }
                        else robot.driveMixer(0,0,0);
                        break;
                    case 6: //demo mode
                        demo();
                        break;
                    case 7: //autonomous demo mode

                }
                robot.updateSensors();
            }

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }

    public void demo(){
//        if(toggleAllowed(gamepad1.y, 3)){
//            runDemo = !runDemo;
//        }
//        if(runDemo){
//            robot.MaintainHeading(gamepad1.a);
//
//            if(gamepad1.b){ autonomous(.20); }
//            else { resetAuto(); }
//
//
//        }
//        else{ joystickDrive(); }
        autonomous(.20);

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

        if(!runDemo) robot.driveMixer(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);

        //toggle the particle conveyor on and off - quick and dirty
        if (toggleAllowed(gamepad1.a,0))
        {
            robot.particle.collect();
        }

        if (toggleAllowed(gamepad1.b,1))
        {
            robot.particle.eject();

        }

        if(toggleAllowed(gamepad1.x,2)) {


            robot.particle.fling();

        }
//
//        if(toggleAllowed(gamepad1.y,3)){
//
//            if (robot.particle.isStopped()) {
//                robot.particle.restart();
//            } else {
//                robot.particle.emergencyStop();
//            }
//        }

//        if(toggleAllowed(gamepad1.dpad_down,4)){
//
//            robot.particle.halfCycle();
//
//        }

//        if(toggleAllowed(gamepad1.dpad_up, 5)){
//            runDemo = !runDemo;
//            robot.resetBeaconPresserState();
//        }
//        if(toggleAllowed(gamepad1.dpad_left, 6) && !runDemo){
//            isBlue = !isBlue;
//        }
//        if(toggleAllowed(gamepad1.dpad_right, 7) && !runDemo){
//            runBeaconTestLeft = !runBeaconTestLeft;
//        }
//
//        if(runDemo) {
//            if(robot.pressAllianceBeacon(isBlue, runBeaconTestLeft))
//                runDemo = false;
//            telemetry.addData("Status", "Test beacon on left Side: " + runBeaconTestLeft);
//            telemetry.addData("Status", "Side: " + getAlliance());
//        }
//        if(toggleAllowed(gamepad1.dpad_up, 5)) {
//            robot.setHeading(90);
//        }
//
//        if(toggleAllowed(gamepad1.dpad_left, 6)) {
//            runDemo = !runDemo;
//        }
//
//        if(runDemo){
//            robot.RotateIMU(.01, 0, 0, Integer.MAX_VALUE, 0);
//        }
        if(gamepad1.dpad_up)
            robot.cap.raise(1);

        else if(gamepad1.dpad_down)
            robot.cap.lower(1);

        else
            robot.cap.stop();
        robot.particle.updateCollection();
    }

    public void resetAuto(){
        autoState = 0;
        robot.ResetTPM();
    }

    public void autonomous(double scaleFactor){
        if(autoState == 0)
            autoTimer = System.nanoTime() + (long) 30e9;
        if(autoTimer > System.nanoTime()) {
            switch (autoState) {
                case -1: //sit and spin - do nothing
                    break;

                case 0: //reset all the motors before starting autonomous
                    //autoTimer = System.nanoTime() + (long) 30e9;
                    robot.setTPM_Forward((long)(robot.getTPM_Forward() * scaleFactor));
                    robot.setTPM_Strafe((long)(robot.getTPM_Strafe() * scaleFactor));
                    robot.resetMotors(true);
                    autoState++;
                    deadShotSays.play(hardwareMap.appContext, R.raw.a01);
                    if(isBlue)
                        robot.setHeading(90);
                    else
                        robot.setHeading(0);
                    break;
                case 1: //drive forward and shoot in the goal

                    if (robot.driveForward(true, .65, 1)) {
                        robot.resetMotors(true);
                        for (int n = 0; n < flingNumber; n++)
                            robot.particle.fling();
                        autoState++;
                        deadShotSays.play(hardwareMap.appContext, R.raw.a02);
                    }
                    break;
                case 2:  // 180 degree turn if alternate alliance, then moves are inverted
                    if(isBlue) {
                        if (robot.RotateIMU(90, 3))
                            autoState++;
                    }
                    else autoState++;
                    break;
                case 3: //drive towards the corner vortex

                    if (robot.driveStrafe(true, .75, 1)) {
                        robot.resetMotors(true);
                        deadShotSays.play(hardwareMap.appContext, R.raw.a04);
                        autoState++;
                    }
                    break;
                case 4: //drive up near the first beacon

                    if (robot.driveForward(!isBlue, 1.5 , 1)) {
                        robot.resetMotors(true);
                        autoState++;
                    }
                    break;
                case 5: //drive towards the wall in order to press the first beacon
                    if (robot.driveStrafe(true, .75 , 1)) {
                        robot.resetMotors(true);
                        autoState++;
                    }
                    break;
                case 6: //press the first beacon
                    if (robot.pressAllianceBeacon(isBlue, false)) {
                        robot.resetMotors(true);
                        autoState++;
                    }
                    break;
                case 7: //drive up next to the second beacon
                    if (robot.driveForward(!isBlue, 1.0, 1)) {
                        robot.resetMotors(true);
                        autoState++;
                    }
                    break;
                case 8: //press the second beacon
                    if (robot.pressAllianceBeacon(isBlue, true)) {
                        robot.resetMotors(true);
                        autoState++;
                    }
                    break;
//            case 9: //drive away from the second beacon
//                if(driveCrab(false, 0, .5)) {
//                    resetMotors();
//                    autoState++;
//                }

                default:
                    robot.ResetTPM();
                    break;
            }
            robot.particle.updateCollection();
        }
        else{
            robot.particle.emergencyStop();
            robot.driveMixer(0, 0, 0);
        }
    }

    public void stateSwitch(){

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

        if(toggleAllowed(gamepad1.left_bumper,8)) {

            state--;
            if (state < 0) {
                state = 5 ;
            }
            robot.resetMotors(true);
            active = false;

        }

        if (toggleAllowed(gamepad1.right_bumper,9)) {

            state++;
            if (state > 5) {
                state = 0;
            }
            robot.resetMotors(true);
            active = false;

        }

        if(toggleAllowed(gamepad1.start,10)) {

            robot.resetMotors(true);
            active = !active;

        }
    }

    public void secondaryAuto(){
        switch(autoState){
            case 0:
                robot.setHeading(135);
                robot.resetMotors(true);
                autoState++;
                break;
            case 1:
                if(robot.driveForward(true, 1.75, 1)){
                    robot.resetMotors(true);
                    autoState++;
                }
                break;
            case 2:
                for(int n = 0; n < flingNumber; n++){
                    robot.particle.fling();
                }
                autoState++;
                robot.motorConveyor.setPower(90);
                break;
            case 3:
                if(robot.driveForward(true, .5, 1)){
                    robot.resetMotors(true);
                    autoState++;
                }
            default:
                break;
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
                });

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
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        //return formatAngle(angles.angleUnit, angles.firstAngle);
                        return Double.toString(robot.getHeading());
                    }
                })
                .addData("headingRaw", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);

                    }
                })
                .addData("headingOffset", new Func<String>() {
                    @Override public String value() {
                        return Double.toString(robot.offsetHeading);

                    }
                })

                .addData("rollRaw", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitchRaw", new Func<String>() {
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
                        return Long.toString(robot.motorFrontLeft.getCurrentPosition());
                    }
                })
                .addData("TicksBL", new Func<String>() {
                    @Override public String value() {
                        return Long.toString(robot.motorBackLeft.getCurrentPosition());
                    }
                })
                .addData("TicksAvg", new Func<String>() {
                    @Override public String value() {
                        return Long.toString(robot.getAverageTicks());
                    }
                });
        telemetry.addLine()
                .addData("DistRear", new Func<String>() {
                    @Override public String value() {
                        return String.valueOf(robot.beaconDistAft);
                    }
                })
                .addData("RearColor", new Func<String>() {
                    @Override public String value() {
                        return Long.toString(robot.colorAft);
                    }
                })
                .addData("DistFore", new Func<String>() {
                    @Override public String value() {
                        return String.valueOf(robot.beaconDistFore);
                    }
                })
                .addData("ForeColor", new Func<String>() {
                    @Override public String value() {
                        return Long.toString(robot.colorFore);
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
