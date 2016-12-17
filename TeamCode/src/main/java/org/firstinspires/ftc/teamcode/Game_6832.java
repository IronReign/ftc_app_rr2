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

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.ftccommon.SoundPlayer;

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

@TeleOp(name="Game_6832", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//  @Autonomous

public class Game_6832 extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    SoundPlayer deadShotSays = SoundPlayer.getInstance();

    //Pele test motors

    DcMotor motorFrontLeft = null;
    DcMotor motorFrontRight = null;
    DcMotor motorBackLeft = null;
    DcMotor motorBackRight = null;
    DcMotor motorConveyor = null;
    DcMotor motorFlinger = null;
    Servo   pushButton = null;

    BNO055IMU imu;
    Orientation angles;

    OpticalDistanceSensor beaconPresentRear;
    OpticalDistanceSensor beaconPresentFore;

    byte[] colorForeCache = new byte[100];
    byte[] colorRearCache = new byte[100];

    I2cDevice beaconColorFore;
    I2cDevice beaconColorRear;
    I2cDeviceSynch colorForeReader;
    I2cDeviceSynch colorRearReader;
    long colorFore;
    long colorAft;
    double beaconDistAft; //holds most recent linearized distance reading from ODS sensor
    double beaconDistFore;

    private double powerFrontLeft = 0;
    private double powerFrontRight = 0;
    private double powerBackLeft = 0;
    private double powerBackRight = 0;
    private double powerConveyor = 0;
    private boolean active = true;
    private scoringSystem kobe = null;
    private long flingTimer = 0;
    private int flingSpeed = 5000; //ticks per second
    private int TPM_Forward = 1772; //set this value
    private int TPM_Crab = 3386; //set this value
    static final private long toggleLockout = (long)2e8; // fractional second lockout between all toggle button
    private long toggleOKTime = 0; //when should next toggle be allowed
    private int autoState = 0;
    private int beaconState = 0;
    private boolean initiallized = false;
    private int flingNumber = 2;
    private boolean isBlue = false;
    private boolean targetBeacon = true;
    private double IMUTargetHeading = 0;
    private boolean targetAngleInitialized = false;
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
        this.pushButton = this.hardwareMap.servo.get("mjolnir");

        // get a reference to our distance sensors
        beaconPresentRear = hardwareMap.opticalDistanceSensor.get("beaconPresentRear");
        beaconPresentFore = hardwareMap.opticalDistanceSensor.get("beaconPresent");

        // color sensors init
        beaconColorFore = hardwareMap.i2cDevice.get("beaconColorFore");
        beaconColorRear = hardwareMap.i2cDevice.get("beaconColorRear");

        colorForeReader = new I2cDeviceSynchImpl(beaconColorFore, I2cAddr.create8bit(0x60), false);
        colorRearReader = new I2cDeviceSynchImpl(beaconColorRear, I2cAddr.create8bit(0x64), false);

        colorForeReader.engage();
        colorRearReader.engage();

        //motor configurations

        this.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorFlinger.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        this.motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motorConveyor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motorFlinger.setDirection(DcMotorSimple.Direction.REVERSE);

        this.kobe = new scoringSystem(flingSpeed, motorFlinger, motorConveyor);

        BNO055IMU.Parameters parametersIMU = new BNO055IMU.Parameters();
        parametersIMU.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parametersIMU.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersIMU.loggingEnabled      = true;
        parametersIMU.loggingTag          = "IMU";

        //imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu = (BNO055IMU)hardwareMap.get("imu");
        imu.initialize(parametersIMU);
        configureDashboard();

        //Set the MR color sensors to passive mode - NEVER DO THIS IN A LOOP - LIMITED NUMBER OF MODE WRITES TO DEVICE
        colorForeReader.write8(3, 1);    //Set the mode of the color sensor using LEDState
        colorRearReader.write8(3, 1);    //Set the mode of the color sensor using LEDState

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

                    kobe.halfCycle();

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
                        kobe.halfCycle();
                        active = false;
                        break;
                    case 3:
                        secondaryAuto();
                        break;
                }
                updateSensors();
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
                    resetMotors();
                    autoState++;
                    deadShotSays.play(hardwareMap.appContext, R.raw.a01);
                    break;
                case 1: //drive forward and shoot in the goal

                    if (driveForward(true, 1.45, 1)) {
                        resetMotors();
                        for (int n = 0; n < flingNumber; n++)
                            kobe.fling();
                        autoState++;
                        deadShotSays.play(hardwareMap.appContext, R.raw.a02);
                    }
                    break;
                case 2:  // 180 degree turn if alternate alliance, then moves are inverted
                    if (isBlue) {

                        if (rotateRelative(true, 90, .30)) {
                            targetAngleInitialized = false;
                            resetMotors();
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

                    if (driveCrab(true, 1, 1)) {
                        resetMotors();
                        deadShotSays.play(hardwareMap.appContext, R.raw.a04);
                        autoState++;
                    }
                    break;
                case 4: //drive up near the first beacon

                    if (driveForward(!isBlue, .20, 1)) {
                        resetMotors();
                        autoState++;
                    }
                    break;
                case 5: //drive towards the wall in order to press the first beacon
                    if (driveCrab(true, .5, 1)) {
                        resetMotors();
                        autoState++;
                    }
                    break;
                case 6: //press the first beacon
                    if (pressTeamBeacon()) {
                        resetMotors();
                        autoState++;
                    }
                    break;
                case 7: //drive up next to the second beacon
                    if (driveForward(!isBlue, 1.0, 1)) {
                        resetMotors();
                        autoState++;
                    }
                    break;
                case 8: //press the second beacon
                    if (pressTeamBeacon()) {
                        resetMotors();
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
            kobe.updateCollection();
        }
        else{
            kobe.emergencyStop();
            driveMixer(0, 0, 0);
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
                resetMotors();
                autoState++;
                break;
            case 1:
                if(driveForward(true, 2.25, 1)){
                    resetMotors();
                    autoState++;
                }
                break;
            case 2:
                for(int n = 0; n < flingNumber; n++){
                    kobe.fling();
                }
                autoState++;
                motorConveyor.setPower(0);
                break;
            case 3:
                if(driveForward(true, .5, 1)){
                    resetMotors();
                    autoState++;
                }
            default:
                break;
        }
    }

    public void updateSensors(){
        // read color sensors
        colorForeCache = colorForeReader.read(0x04, 1);
        colorRearCache = colorRearReader.read(0x04, 1);
        colorAft  = (colorRearCache[0] & 0xFF);
        colorFore = (colorForeCache[0] & 0xFF);

        //odsReadingLinear = Math.pow(odsReadingRaw, 0.5);
        beaconDistAft  = Math.pow(beaconPresentRear.getLightDetected(), 0.5); //calculate linear value
        beaconDistFore = Math.pow(beaconPresentFore.getLightDetected(), 0.5); //calculate linear value
    }

    public void joystickDrive(){


        driveMixer(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);

        motorFrontLeft.setPower(powerFrontLeft);
        motorBackLeft.setPower(powerBackLeft);
        motorFrontRight.setPower(powerFrontRight);
        motorBackRight.setPower(powerBackRight);

        //toggle the particle conveyor on and off - quick and dirty
        if (toggleAllowed(gamepad1.a,0))
        {
                kobe.collect();
        }

        if (toggleAllowed(gamepad1.b,1))
        {
                kobe.eject();

        }

        if(toggleAllowed(gamepad1.x,2)) {


                kobe.fling();

        }

        if(toggleAllowed(gamepad1.y,3)){

                if (kobe.isStopped()) {
                    kobe.restart();
                } else {
                    kobe.emergencyStop();
                }
        }

        if(toggleAllowed(gamepad1.dpad_down,4)){

                kobe.halfCycle();

        }

        kobe.updateCollection();
    }

    public double clampMotor(double power) { return clampDouble(-1, 1, power); }



    public boolean foundBeacon() {
        if(isBlue){
            return beaconDistAft > 0.05;
        }
        return beaconDistFore > .05;
    }

    public boolean findBeaconPressRange() {
        double dist;
        if(isBlue){ dist = beaconDistAft; }
        else { dist = beaconDistFore; }
        if(dist > .25){
            driveMixer(0, -.35, 0);
            return false;
        }
        else if(dist < .15){
            driveMixer(0, .35, 0);
            return false;
        }
        else{
            driveMixer(0, 0, 0);
            return true;
        }
    }

    public boolean onTeamColor(){
        if(isBlue){
            return colorAft == 3;
        }
        return (colorFore > 9 && colorFore < 12);
    }

    public boolean pressTeamBeacon(){
        switch(beaconState){
            case 0:
                if(isBlue){ driveMixer(-1, 0, 0); }
                else { driveMixer(1, 0, 0); }
                if(foundBeacon()) {
                    driveMixer(0, 0, 0);
                    resetMotors();
                    beaconState++;
                }
                break;
            case 1:
                if(driveForward(!isBlue, .10, .45)){
                    resetMotors();
                    beaconState++;
                }
            case 2:
                if(findBeaconPressRange()) beaconState++;
                break;
            case 3:
                if(findBeaconPressRange()) beaconState++;
                break;
            case 4:
                if(isBlue){ driveMixer(-.35, 0, 0); }
                else { driveMixer(.35, 0, 0); }
                if(onTeamColor()) beaconState++;
                break;
            case 5:
                if(driveForward(true, 0, .25)){
                    resetMotors();
                    beaconState++;
                }
                break;
            case 6:
                pushButton.setPosition(ServoNormalize(pressedPosition));
                presserTimer = System.nanoTime() + (long) 2e9;
                beaconState++;
                break;
            case 7:
                if(presserTimer < System.nanoTime())
                    beaconState++;
                break;
            case 8:
                pushButton.setPosition(ServoNormalize(relaxedPosition));
                beaconState++;
                break;
            case 9:
                if(driveCrab(false, .10, .50)) beaconState++;
                break;
            default:
                beaconState = 0;
                return true;
        }
        return false;
    }

    public double clampDouble(double min, double max, double value)
    {
        double result = value;
        if(value > max)
            result = max;
        if(value < min)
            result = min;
        return result;
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

        motorFrontLeft.setPower(clampMotor(powerFrontLeft));
        motorBackLeft.setPower(clampMotor(powerBackLeft));
        motorFrontRight.setPower(clampMotor(powerFrontRight));
        motorBackRight.setPower(clampMotor(powerBackRight));

    }
//    public void moveTicks(double forward, double crab, double rotate, long ticks){
//        ticks += motorFrontLeft.getCurrentPosition();
//        while(motorFrontLeft.getCurrentPosition() < ticks && opModeIsActive()){
//            telemetry.addData("Status", "Front Left Ticks: " + Long.toString(motorFrontLeft.getCurrentPosition()));
//            telemetry.update();
//            driveMixer(forward, crab, rotate);
//        }
//    }
    public boolean driveForward(boolean forward, double targetMeters, double power){
        if(!forward){
            targetMeters = 0 - targetMeters;
            power = -power;
        }
        long targetPos = (long)(targetMeters * TPM_Forward);
        if(Math.abs(targetPos) > Math.abs(getAverageTicks())){
            driveMixer(power, 0, 0);
            return false;
        }
        else {
            driveMixer(0, 0, 0);
            return true;
        }
    }
    boolean rotateRelative(boolean clockwise, double targetAngle, double power){
        if(!clockwise){
            targetAngle = -targetAngle;
            power = -power;
        }
        if(!targetAngleInitialized) { targetAngle = targetAngle + angles.firstAngle; targetAngleInitialized = true; }
        if(Math.abs(targetAngle) > Math.abs(angles.firstAngle)){
            driveMixer(0, 0, power);
            return false;
        }
        else {
            driveMixer(0, 0, 0);
            return true;
        }
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

    public boolean driveCrab(boolean left, double targetMeters, double power){
        if(!left){
            targetMeters = -targetMeters;
            power = -power;
        }
        long targetPos = (long)(targetMeters * TPM_Crab);
        if(Math.abs(targetPos) > Math.abs(getAverageAbsTicks())){
            driveMixer(0, power, 0);
            return false;
        }
        else {
            driveMixer(0, 0, 0);
            return true;
        }
    }
    public void resetMotors(){
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public long getAverageTicks(){
        long averageTicks = (motorFrontLeft.getCurrentPosition() + motorBackLeft.getCurrentPosition() + motorFrontRight.getCurrentPosition() + motorBackRight.getCurrentPosition())/4;
        return averageTicks;
    }
    public long getAverageAbsTicks(){
        long averageTicks = (Math.abs(motorFrontLeft.getCurrentPosition()) + Math.abs(motorBackLeft.getCurrentPosition()) + Math.abs(motorFrontRight.getCurrentPosition()) + Math.abs(motorBackRight.getCurrentPosition()))/4;
        return averageTicks;
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
                angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
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
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
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
                        return Long.toString(motorFrontLeft.getCurrentPosition());
                    }
                })
                .addData("TicksAvg", new Func<String>() {
                    @Override public String value() {
                        return Long.toString(getAverageTicks());
                    }
                });
        telemetry.addLine()
                .addData("DistRear", new Func<String>() {
                    @Override public String value() {
                        return String.valueOf(beaconDistAft);
                    }
                })
                .addData("RearColor", new Func<String>() {
                    @Override public String value() {
                        return Long.toString(colorAft);
                    }
                })
                .addData("DistFore", new Func<String>() {
                    @Override public String value() {
                        return String.valueOf(beaconDistFore);
                    }
                })
                .addData("ForeColor", new Func<String>() {
                    @Override public String value() {
                        return Long.toString(colorFore);
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
