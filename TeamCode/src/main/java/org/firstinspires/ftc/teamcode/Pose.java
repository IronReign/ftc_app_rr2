package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.adafruit.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;


/**
 * The Pose class stores the current real world position/orientation: <b>position</b>, <b>heading</b>,
 * and <b>speed</b> of the robot.
 *
 * This class should be a point of reference for any navigation classes that want to know current
 * orientation and location of the robot.  The update method must be called regularly, it
 * monitors and integrates data from the orientation (IMU) and odometry (motor encoder) sensors.
 * @author Max Virani, Tycho Virani
 * @version 1.2
 * @since 2016-12-10
 */

public class Pose
{

    HardwareMap hwMap;

    //motors

    PIDController drivePID = new PIDController(0, 0, 0);

    protected double KpDrive = 0.007;
    private double KiDrive = 0.00;
    private double KdDrive = 0.001;
    private double driveIMUBasePower = .5;
    private double motorPower = 0;

    DcMotor motorFrontLeft  = null;
    DcMotor motorFrontRight = null;
    DcMotor motorBackLeft   = null;
    DcMotor motorBackRight  = null;
    DcMotor motorConveyor   = null;
    DcMotor motorLauncher   = null;
    DcMotor motorLift       = null;
    Servo servoGate         = null;

    BNO055IMU imu;
    Orientation angles;

    OpticalDistanceSensor beaconPresentRear;
    OpticalDistanceSensor beaconPresent;


    byte[] colorForeCache = new byte[100];
    byte[] colorRearCache = new byte[100];

    I2cDevice beaconColor;
    I2cDevice ballColor;
    I2cDeviceSynch colorForeReader;
    I2cDeviceSynch colorRearReader;
    long colorFore;
    long colorAft;
    double beaconDistAft; //holds most recent linearized distance reading from ODS sensor
    double beaconDistFore;

    private double powerFrontLeft  = 0;
    private double powerFrontRight = 0;
    private double powerBackLeft   = 0;
    private double powerBackRight  = 0;
    private double powerConveyor   = 0;

    public ParticleSystem particle = null;
    public CapTrap cap = null;

    private long flingTimer = 0;
    private int flingSpeed  = 5000; //ticks per second
    private int TPM_Forward = 2439; //measurement was for the original 42 tooth driven sprocket, since replaced by a 32 tooth sprocket
    private int TPM_Strafe  = 3145 ; //measurement was for the original 42 tooth driven sprocket, since replaced by a 32 tooth sprocket

    private double poseX;
    private double poseY;
    private double poseHeading;
    private double poseHeadingRad; //current heading converted to radians. Might be rotated by 90 degrees from imu's heading when strafing
    private double poseSpeed;
    private double posePitch;
    private double poseRoll;
    private long timeStamp; //timestamp of last update
    private boolean initialized = false;
    public  double offsetHeading;
    private double offsetPitch;
    private double offsetRoll;
    private double displacement;
    private double displacementPrev;
    private double odometer;
    static double scanSpeed = .25;
    private long presserTimer = 0;
    private long presserSavedTime = 0;
    private double zeroHeading = 0;
    private long turnTimer = 0;
    private boolean turnTimerInit = false;
    private double minTurnError = 1.0;
    public boolean maintainHeadingInit = false;;
    private double poseSavedHeading = 0.0;

    SoundPlayer deadShotSays = SoundPlayer.getInstance();

    private long ticksLeftPrev;
    private long ticksRightPrev;
    private long ticksLeftOffset; //provide a way to offset (effectively reset) the motor encoder readings
    private long ticksRightOffset;
    private double wheelbase; //the width between the wheels

    final protected double flingerRelaxPwr = 0.075;
    final protected double flingerFlingPwr = -1;

    long flingerTimer;

    public enum MoveMode{
        forward,
        backward,
        left,
        right,
        rotate,
        still;
    }

    protected MoveMode moveMode;



    Orientation imuAngles;
    protected boolean targetAngleInitialized = false;
    private int beaconState = 0;


    /**
     * Create a Pose instance that stores all real world position/orientation elements:
     * <var>x</var>, <var>y</var>, <var>heading</var>, and <var>speed</var>.
     *
     * @param x     The position relative to the x axis of the field
     * @param y     The position relative to the y axis of the field
     * @param heading The heading of the robot
     * @param speed The speed of the robot
     */
    public Pose(double x, double y, double heading, double speed)
    {

        poseX     = x;
        poseY     = y;
        poseHeading = heading;
        poseSpeed = speed;
        posePitch = 0;
        poseRoll = 0;
    }

    /**
     * Creates a Pose instance with _0 speed, to prevent muscle fatigue
     * by excess typing demand on the software team members.
     *
     * @param x     The position relative to the x axis of the field
     * @param y     The position relative to the y axis of the field
     * @param angle The angle of the robot
     */
    public Pose(double x, double y, double angle)
    {

        poseX     = x;
        poseY     = y;
        poseHeading = angle;
        poseSpeed = 0;

    }

    /**
     * Creates a base Pose instance at the origin, (_0,_0), with _0 speed and _0 angle.
     * Useful for determining the Pose of the robot relative to the origin.
     */
    public Pose()
    {

        poseX     = 0;
        poseY     = 0;
        poseHeading = 0;
        poseSpeed = 0;
        posePitch=0;
        poseRoll=0;

    }

    public void ResetTPM(){
        TPM_Forward = 2493;
        TPM_Strafe = 3145;
    }


    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;
               /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        this.motorFrontLeft  = this.hwMap.dcMotor.get("motorFrontLeft");
        this.motorFrontRight = this.hwMap.dcMotor.get("motorFrontRight");
        this.motorBackLeft   = this.hwMap.dcMotor.get("motorBackLeft");
        this.motorBackRight  = this.hwMap.dcMotor.get("motorBackRight");
        this.motorConveyor   = this.hwMap.dcMotor.get("motorConveyor");
        this.motorLauncher   = this.hwMap.dcMotor.get("motorLauncher");
        this.motorLift       = this.hwMap.dcMotor.get("motorLift");
        this.servoGate       = this.hwMap.servo.get("servoGate");

        // get a reference to our distance sensors
        beaconPresentRear = hwMap.opticalDistanceSensor.get("beaconPresentRear");
        beaconPresent = hwMap.opticalDistanceSensor.get("beaconPresentFore");

        // color sensors init
        beaconColor = hwMap.i2cDevice.get("beaconColor");
        ballColor = hwMap.i2cDevice.get("ballColor");

        colorForeReader = new I2cDeviceSynchImpl(beaconColor, I2cAddr.create8bit(0x60), false);
        colorRearReader = new I2cDeviceSynchImpl(ballColor, I2cAddr.create8bit(0x64), false);

        colorForeReader.engage();
        colorRearReader.engage();

        //motor configurations

        this.motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motorConveyor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motorLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motorLift.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motorLauncher.setDirection(DcMotorSimple.Direction.REVERSE);

        moveMode = MoveMode.still;

        this.particle = new ParticleSystem(flingSpeed, motorLauncher, motorConveyor, servoGate);
        this.cap = new CapTrap(motorLift);

        BNO055IMU.Parameters parametersIMU = new BNO055IMU.Parameters();
        parametersIMU.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parametersIMU.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersIMU.loggingEnabled       = true;
        parametersIMU.loggingTag           = "IMU";

        //imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu = (BNO055IMU)hwMap.get("imu");
        imu.initialize(parametersIMU);


        //Set the MR color sensors to passive mode - NEVER DO THIS IN A LOOP - LIMITED NUMBER OF MODE WRITES TO DEVICE
        colorForeReader.write8(3, 1);    //Set the mode of the color sensor using LEDState
        colorRearReader.write8(3, 1);    //Set the mode of the color sensor using LEDState

    }

    public void DrivePID(double Kp, double Ki, double Kd, double pwr, double currentAngle, double targetAngle) {
        //if (pwr>0) PID.setOutputRange(pwr-(1-pwr),1-pwr);
        //else PID.setOutputRange(pwr - (-1 - pwr),-1-pwr);
        drivePID.setOutputRange(-.5,.5);
        drivePID.setPID(Kp, Ki, Kd);
        drivePID.setSetpoint(targetAngle);
        drivePID.enable();

        drivePID.setInputRange(0, 360);
        drivePID.setContinuous();
        drivePID.setInput(currentAngle);
        double correction = drivePID.performPID();
        /*ArrayList toUpdate = new ArrayList();
        toUpdate.add((PID.m_deltaTime));
        toUpdate.add(Double.valueOf(PID.m_error));
        toUpdate.add(new Double(PID.m_totalError));
        toUpdate.add(new Double(PID.pwrP));
        toUpdate.add(new Double(PID.pwrI));
        toUpdate.add(new Double(PID.pwrD));*/
/*
        logger.UpdateLog(Long.toString(System.nanoTime()) + ","
                + Double.toString(PID.m_deltaTime) + ","
                + Double.toString(pose.getOdometer()) + ","
                + Double.toString(PID.m_error) + ","
                + Double.toString(PID.m_totalError) + ","
                + Double.toString(PID.pwrP) + ","
                + Double.toString(PID.pwrI) + ","
                + Double.toString(PID.pwrD) + ","
                + Double.toString(correction));
        motorLeft.setPower(pwr + correction);
        motorRight.setPower(pwr - correction);
*/
        driveMixer(pwr, 0, correction);
    }



    public void MovePID(double Kp, double Ki, double Kd, double pwr, double currentAngle, double targetAngle) {
        //if (pwr>0) PID.setOutputRange(pwr-(1-pwr),1-pwr);
        //else PID.setOutputRange(pwr - (-1 - pwr),-1-pwr);
        drivePID.setOutputRange(-.5,.5);
        drivePID.setPID(Kp, Ki, Kd);
        drivePID.setSetpoint(targetAngle);
        drivePID.enable();

        drivePID.setInputRange(0, 360);
        drivePID.setContinuous();
        drivePID.setInput(currentAngle);
        double correction = drivePID.performPID();
        /*ArrayList toUpdate = new ArrayList();
        toUpdate.add((PID.m_deltaTime));
        toUpdate.add(Double.valueOf(PID.m_error));
        toUpdate.add(new Double(PID.m_totalError));
        toUpdate.add(new Double(PID.pwrP));
        toUpdate.add(new Double(PID.pwrI));
        toUpdate.add(new Double(PID.pwrD));*/
/*
        logger.UpdateLog(Long.toString(System.nanoTime()) + ","
                + Double.toString(PID.m_deltaTime) + ","
                + Double.toString(pose.getOdometer()) + ","
                + Double.toString(PID.m_error) + ","
                + Double.toString(PID.m_totalError) + ","
                + Double.toString(PID.pwrP) + ","
                + Double.toString(PID.pwrI) + ","
                + Double.toString(PID.pwrD) + ","
                + Double.toString(correction));
        motorLeft.setPower(pwr + correction);
        motorRight.setPower(pwr - correction);

*/
        driveMixer(pwr, 0, correction);
    }
    public void DriveIMU(double Kp, double Ki, double Kd, double pwr, double targetAngle){
        MovePID(Kp, Ki, Kd, pwr, poseHeading, targetAngle);
    }

    public boolean DriveIMUDistance(double Kp, double pwr, double targetAngle,boolean forward, double targetMeters){
        if(!forward){
            moveMode = moveMode.backward;
            targetMeters = -targetMeters;
            pwr = -pwr;
        }
        else moveMode = moveMode.forward;

        long targetPos = (long)(targetMeters * TPM_Forward);
        if(Math.abs(targetPos) > Math.abs(getAverageTicks())){//we've not arrived yet
            DriveIMU(Kp, KiDrive, KdDrive, pwr, targetAngle);
            return false;
        }
        else { //destination achieved
            driveMixer(0, 0, 0);
            return true;
        }
    }

    public boolean RotateIMU(double targetAngle, double maxTime){ //uses default pose PID constants and has end conditions
        if(!turnTimerInit){
            turnTimer = System.nanoTime() + (long)(maxTime * (long) 1e9);
            turnTimerInit = true;
        }
        DriveIMU(KpDrive, KiDrive, KdDrive, 0, targetAngle); //if the robot turns within a threshold of the target
        if(Math.abs(poseHeading - targetAngle) < minTurnError) {
            turnTimerInit = false;
            driveMixer(0,0,0);
            return true;
        }
        if(turnTimer < System.nanoTime()){ //if the robot takes too long to turn within a threshold of the target (it gets stuck)
            turnTimerInit = false;
            driveMixer(0,0,0);
            return true;
        }
        return false;
    }
    public void MaintainHeading(boolean buttonState){
        if(buttonState) {
            if (!maintainHeadingInit) {
                poseSavedHeading = poseHeading;
                maintainHeadingInit = true;
            }
            DriveIMU(KpDrive, KiDrive, KdDrive, 0, poseSavedHeading);
        }
        if(!buttonState){
            maintainHeadingInit = false;
        }
    }

    public void setZeroHeading(){
        setHeading(0);
    }
    public void setHeading(double angle){
        poseHeading = angle;
        initialized = false;
    }

    public void driveMixer(double forward,double strafe ,double rotate){
        powerBackRight = 0;
        powerFrontRight = 0;
        powerBackLeft = 0;
        powerFrontLeft = 0;

        powerFrontLeft = forward;
        powerBackLeft = forward;
        powerFrontRight = forward;
        powerBackRight = forward;

        powerFrontLeft += -strafe;
        powerFrontRight += strafe;
        powerBackLeft += strafe;
        powerBackRight += -strafe;

        powerFrontLeft -= rotate;
        powerBackLeft -= rotate;
        powerFrontRight += rotate;
        powerBackRight += rotate;

        motorFrontLeft.setPower(clampMotor(powerFrontLeft));
        motorBackLeft.setPower(clampMotor(powerBackLeft));
        motorFrontRight.setPower(clampMotor(powerFrontRight));
        motorBackRight.setPower(clampMotor(powerBackRight));

    }

    public void resetMotors(boolean enableEncoders){
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (enableEncoders) {
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        else {
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

    }

    public long getAverageTicks(){
        long averageTicks = (motorFrontLeft.getCurrentPosition() + motorBackLeft.getCurrentPosition() + motorFrontRight.getCurrentPosition() + motorBackRight.getCurrentPosition())/4;
        return averageTicks;
    }
    public long getAverageAbsTicks(){
        long averageTicks = (Math.abs(motorFrontLeft.getCurrentPosition()) + Math.abs(motorBackLeft.getCurrentPosition()) + Math.abs(motorFrontRight.getCurrentPosition()) + Math.abs(motorBackRight.getCurrentPosition()))/4;
        return averageTicks;
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



    //    public void moveTicks(double forward, double strafe, double rotate, long ticks){
//        ticks += motorFrontLeft.getCurrentPosition();
//        while(motorFrontLeft.getCurrentPosition() < ticks && opModeIsActive()){
//            telemetry.addData("Status", "Front Left Ticks: " + Long.toString(motorFrontLeft.getCurrentPosition()));
//            telemetry.update();
//            driveMixer(forward, strafe, rotate);
//        }
//    }
    public boolean driveForward(boolean forward, double targetMeters, double power){
        if(!forward){
            moveMode = moveMode.backward;
            targetMeters = -targetMeters;
            power = -power;
        }
        else moveMode = moveMode.forward;

        long targetPos = (long)(targetMeters * TPM_Forward);
        if(Math.abs(targetPos) > Math.abs(getAverageTicks())){//we've not arrived yet
            driveMixer(power, 0, 0);
            return false;
        }
        else { //destination achieved
            driveMixer(0, 0, 0);
            return true;
        }
    }
    boolean rotateRelative(boolean clockwise, double targetAngle, double power){
        moveMode = moveMode.rotate;
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

    public boolean driveStrafe(boolean left, double targetMeters, double power){

        if(!left){
            moveMode = moveMode.right;
            targetMeters = -targetMeters;
            power = -power;
        }
        else moveMode = moveMode.left;

        long targetPos = (long)(targetMeters * TPM_Strafe);
        if(Math.abs(targetPos) > Math.abs(getAverageAbsTicks())){
            driveMixer(0, power, 0);
            return false;
        }
        else {
            driveMixer(0, 0, 0);
            return true;
        }
    }


    /**
     * Set the current position of the robot in the X direction on the field
     * @param poseX
     */
    public void setPoseX(double poseX) {
        this.poseX = poseX;
    }

    /**
     * Set the current position of the robot in the Y direction on the field
     * @param poseY
     */
    public void setPoseY(double poseY) {
        this.poseY = poseY;
    }

    /**
     * Set the absolute heading (yaw) of the robot _0-360 degrees
     * @param poseHeading
     */
    public void setPoseHeading(double poseHeading) {
        this.poseHeading = poseHeading;
        initialized = false; //trigger recalc of offset on next Update
    }

    /**
     * Set the absolute pitch of the robot _0-360 degrees
     * @param posePitch
     */
    public void setPosePitch(double posePitch) {
        this.posePitch = posePitch;
        initialized = false; //trigger recalc of offset on next Update
    }

    /**
     * Set the absolute roll of the robot _0-360 degrees
     * @param poseRoll
     */
    public void setPoseRoll(double poseRoll) {
        this.poseRoll = poseRoll;
        initialized = false; //trigger recalc of offset on next Update
    }
    /**
     * Returns the x position of the robot
     *
     * @return The current x position of the robot
     */
    public double getX()
    {
        return poseX;
    }

    /**
     * Returns the y position of the robot
     *
     * @return The current y position of the robot
     */
    public double getY()
    {
        return poseY;
    }

    /**
     * Returns the angle of the robot
     *
     * @return The current angle of the robot
     */
    public double getHeading()
    {
        return poseHeading;
    }

    /**
     * Returns the speed of the robot
     *
     * @return The current speed of the robot
     */
    public double getSpeed()
    {
        return poseSpeed;
    }
    public double getPitch() {
        return posePitch;
    }

    public double getRoll() {
        return poseRoll;
    }

    public long getTPM_Forward() {
        return TPM_Forward;
    }

    public void setTPM_Forward(long TPM_Forward) { this.TPM_Forward = (int)TPM_Forward; }

    public long getTPM_Strafe() {
        return TPM_Strafe;
    }

    public void setTPM_Strafe(long TPM_Strafe) { this.TPM_Strafe = (int)TPM_Strafe; }

    public void setTicksPerMeterLeft(long TPM_Strafe) {
        this.TPM_Strafe = (int)TPM_Strafe;
    }

    public void updateSensors(){
        // read color sensors
        colorForeCache = colorForeReader.read(0x04, 1);
        colorRearCache = colorRearReader.read(0x04, 1);
        colorAft  = (colorRearCache[0] & 0xFF);
        colorFore = (colorForeCache[0] & 0xFF);

        //odsReadingLinear = Math.pow(odsReadingRaw, 0.5);
        beaconDistAft  = Math.pow(beaconPresentRear.getLightDetected(), 0.5); //calculate linear value
        beaconDistFore = Math.pow(beaconPresent.getLightDetected(), 0.5); //calculate linear value
        Update(imu, 0, 0);
    }

    /**
     * Update the current location of the robot. This implementation gets heading and orientation
     * from the Bosch BNO055 IMU and assumes a simple differential steer robot with left and right motor
     * encoders.
     *
     *
     * The current naive implementation assumes an unobstructed robot - it cannot account
     * for running into objects and assumes no slippage in the wheel encoders.  Debris
     * on the field and the mountain ramps will cause problems for this implementation. Further
     * work could be done to compare odometry against IMU integrated displacement calculations to
     * detect stalls and slips
     *
     * This method should be called regularly - about every 20 - 30 milliseconds or so.
     *
     * @param imu
     * @param ticksLeft
     * @param ticksRight
     */

    public void Update(BNO055IMU imu, long ticksLeft, long ticksRight){
        long currentTime = System.nanoTime();
        imuAngles= imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        if (!initialized){
            //first time in - we assume that the robot has not started moving and that orientation values are set to the current absolute orientation
            //so first set of imu readings are effectively offsets


            offsetHeading = wrapAngleMinus(poseHeading, imuAngles.firstAngle);
            offsetRoll = wrapAngleMinus(imuAngles.secondAngle, poseRoll);
            offsetPitch = wrapAngleMinus(imuAngles.thirdAngle, posePitch);

            initialized = true;
        }


        poseHeading = wrapAngle(imuAngles.firstAngle, offsetHeading);
        posePitch = wrapAngle(imuAngles.thirdAngle, offsetPitch);
        poseRoll = wrapAngle(imuAngles.secondAngle, offsetRoll);

        //double displacement = (((double)(ticksRight - ticksRightPrev)/ticksPerMeterRight) + ((double)(ticksLeft - ticksLeftPrev)/ticksPerMeterLeft))/2.0;

        // we haven't worked out the trig of calculating displacement from any driveMixer combination, so
        // for now we are just restricting ourselves to cardinal relative directions of pure forward, backward, left and right
        // so no diagonals or rotations - if we do those then our absolute positioning fails

        switch (moveMode) {
            case forward:
            case backward:
                displacement = (getAverageTicks() - displacementPrev) * TPM_Forward;
                odometer += Math.abs(displacement);
                poseHeadingRad = Math.toRadians(poseHeading);
                break;
            case left:
                displacement = (getAverageAbsTicks() - displacementPrev) * TPM_Strafe; //todo: there might be a problem when switching between driving forward and strafing - displacementPrev may need to be updated the first time in to use the correct avgTicks calculation
                poseHeadingRad = Math.toRadians(poseHeading)- Math.PI/2; //actual heading is rotated 90 degrees counterclockwise

                break;
            case right:
                displacement = (getAverageAbsTicks() - displacementPrev) * TPM_Strafe; //todo: there might be a problem when switching between driving forward and strafing - displacementPrev may need to be updated the first time in to use the correct avgTicks calculation
                poseHeadingRad = Math.toRadians(poseHeading) + Math.PI/2; //actual heading is rotated 90 degrees clockwise

                break;
            default:
                displacement=0; //when rotating or in an undefined moveMode, ignore/reset displacement
                displacementPrev = 0;
                break;
        }

        odometer += Math.abs(displacement);
        poseSpeed = displacement / (double)(currentTime - this.timeStamp)*1000000; //meters per second when ticks per meter is calibrated

        timeStamp = currentTime;
        displacementPrev = displacement;

        poseX += displacement * Math.cos(poseHeadingRad);
        poseY += displacement * Math.sin(poseHeadingRad);

    }

    public long getTicksLeftPrev()
    {
        return ticksLeftPrev;
    }
    public long getTicksRightPrev()
    {
        return ticksRightPrev;
    }

    /**
     *
     * gets the odometer. The odometer tracks the robot's total amount of travel since the last odometer reset
     * The value is in meters and is always increasing (absolute), even when the robot is moving backwards
     * @returns odometer value
     */
    public double getOdometer() {

        return  odometer;

    }

    /**
     * resets the odometer. The odometer tracks the robot's total amount of travel since the last odometer reset
     * The value is in meters and is always increasing (absolute), even when the robot is moving backwards
     * @param distance
     */
    public void setOdometer(double distance){
        odometer = 0;
    }

    /**
     * returns the minimum difference (in absolute terms) between two angles,
     * preserves the sign of the difference
     *
     * @param angle1
     * @param angle2
     * @return
     */
    public double diffAngle(double angle1, double angle2){
        return Math.abs(angle1 - angle2) < Math.abs(angle2-angle1) ? Math.abs(angle1 - angle2) : Math.abs(angle2-angle1);
    }

    public double diffAngle2(double angle1, double angle2){

        double diff = angle1 - angle2;

        //allow wrap around

        if (Math.abs(diff) > 180)
        {
            if (diff > 0) {
                diff -= 360;
            } else {
                diff += 360;
            }
        }
        return diff;
    }


    /**
     * Apply and angular adjustment to a base angle with result wrapping around at 360 degress
     *
     * @param angle1
     * @param angle2
     * @return
     */
    public double wrapAngle(double angle1, double angle2){
        return (angle1 + angle2) % 360;
    }
    public double wrapAngleMinus(double angle1, double angle2){
        return 360-((angle1 + angle2) % 360);
    }

    double getBearingTo(double x, double y){
        double diffx = x-poseX;
        double diffy = y-poseY;
        return ( Math.toDegrees(Math.atan2( diffy, diffx)) - 90  + 360 ) % 360;
    }

    double getBearingOpposite(double x, double y){
        double diffx = x-poseX;
        double diffy = y-poseY;
        return ( Math.toDegrees(Math.atan2( diffy, diffx)) + 90 + 360 ) % 360;
    }

    double getDistanceTo(double x, double y){

        double dx = x - poseX;
        double dy = y - poseY;
        return Math.sqrt(dx*dx + dy*dy);

    }


    public double ServoNormalize(int pulse){
        double normalized = (double)pulse;
        return (normalized - 750.0) / 1500.0; //convert mr servo controller pulse width to double on _0 - 1 scale
    }

    public boolean nearBeacon(boolean isBlue) { //did the optical distance sensor see something close enough to
        if(isBlue){                              //be one of the beacons
            return beaconDistFore > 0.08;
        }
        return beaconDistFore > .08;
    }

    public boolean findBeaconPressRange(boolean isBlue) { //is the robot close enough to push the beacon with the
        double dist;                                      //the servo (DEPRECATED)
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

    public void drivePID(boolean forward, double power){

    }

    public boolean onAllianceColor(boolean isBlue){ //is the robot looking at it's team's aliance color
        if(isBlue){
            return colorFore == 3;
        }
        return (colorFore > 9 && colorFore < 12);
    }

    public boolean onOpposingColor(boolean isBlue){ //is the robot looking at it's team's aliance color
        if(isBlue){
            return (colorFore > 9 && colorFore < 12);
        }
        return colorFore == 3;
    }
    public boolean findOpposingColor(boolean isBlue, boolean fromLeft, double pwr){
        if((isBlue && fromLeft) || (!isBlue && !fromLeft)){ driveMixer(-pwr, 0, 0); }
        else { driveMixer(pwr, 0, 0); }
        if(onOpposingColor(isBlue)){
            return true;
        }
        return false;
    }

    // Don't need this since we now have a PID version of RotateIMU
//    public boolean turnIMU(double targetAngle, double power, boolean turnRight){
//            if(turnRight)
//                driveMixer(0, 0, power);
//            else
//                driveMixer(0, 0, -power);
//            if(turnRight && targetAngle <= poseHeading)
//                return true;
//            else if(!turnRight && targetAngle >= poseHeading)
//                return true;
//            else return false;
//
//    }
    public void driveToBeacon(VuforiaTrackableDefaultListener beacon, double bufferDistance, double speed, boolean strafe) {

        if (beacon.getPose() != null) {
            VectorF trans = beacon.getPose().getTranslation();

            double angle = Math.toDegrees(Math.atan2(trans.get(0), -trans.get(2)));
            Log.i("Beacon Angle", String.valueOf(angle));

            if (strafe) {
                //track(angle, Math.hypot(trans.get(0), trans.get(2) - bufferDistance), speed);
                //driveMixer()
            } else {  //turn first, then drive
                RotateIMU(angle, 1.0);
                //track(0, Math.hypot(trans.get(0), trans.get(2)) - bufferDistance, speed);
            }//else

        } else {
            Log.i("VISION", "drive To Beacon failed: Beacon not visible");
        }//else

    }//driveToBeacon

//    public void driveToBeacon(VuforiaTrackableDefaultListener beacon, double bufferDistance, double speed, boolean strafe,
//                               double robotAngle, VectorF coordinate) {
//
//        if (beacon.getPose() != null) {
//            VectorF trans = beacon.getPose().getTranslation();
//
//            Log.i(TAG, "strafeToBeacon: " + trans);
//
//            trans = VortexUtils.navOffWall(trans, robotAngle, coordinate);
//
//            Log.i(TAG, "strafeToBeacon: " + trans);
//
//            double angle = Math.toDegrees(Math.atan2(trans.get(0), trans.get(2)));
//
//            Log.i(TAG, "strafeToBeacon: " + angle);
//
//
//            if (strafe) {
//                //track(angle, Math.hypot(trans.get(0), trans.get(2)) - bufferDistance, speed);
//            } else {
//                if (angle < 0) {
//                    imuTurnL(-angle, speed);
//                } else {
//                    imuTurnR(angle, speed);
//                }//else
//
//                track(0, Math.hypot(trans.get(0), trans.get(2) - bufferDistance), speed);
//            }//else
//
//        } else {
//            RC.t.addData("FERMION", "Strafe To Beacon failed: Beacon not visible");
//        }//else
//    }//driveToBeacon



    public boolean pressAllianceBeacon(boolean isBlue, boolean fromLeft){ //press the button on the beacon that corresponds
        switch(beaconState){                                              // to the alliance color in tertiaryAu2to
            case 0:
                //if((isBlue && fromLeft) || (!isBlue && !fromLeft)){ driveMixer(-scanSpeed, 0, 0); }
                //else { driveMixer(scanSpeed, 0, 0); }
                //if(nearBeacon(isBlue)) {
                //    driveMixer(0, 0, 0);
                    resetMotors(true);
                    beaconState++;
                //}
                break;
            case 1:     //stub
                //if(driveForward(((isBlue && fromLeft) || (!isBlue && !fromLeft)), .1, .5)){
//                    resetMotors();
                    beaconState++;
                //}
                break;
            case 2:     //stub
//                if(findBeaconPressRange())
                beaconState++;
                break;
            case 3:     //stub
//                if(findBeaconPressRange())
                deadShotSays.play(hwMap.appContext, R.raw.a03);
                beaconState++;
                break;
            case 4:     //drives to find the opposing alliance's color on the beacon in order to put it out of the
                if(findOpposingColor(isBlue, fromLeft, 0.15)) beaconState++;  //range of the beacon presser
                break;
            case 5:     //stub
//                if(driveForward(true, 0, .25)){
//                    resetMotors();
                //check to see if we overshot, if so, go back very slowly and find the color again
                if(onOpposingColor(isBlue)){
                    if(findOpposingColor(isBlue, !fromLeft, 0.10)) beaconState++;
                }
                    else beaconState++;
                break;
            case 6:     //begins moving sideways in order to press the beacon and sets a timer to stop moving if the
                        //beacon takes too long to switch
//                servoGate.setPosition(ServoNormalize(pressedPosition));
//                presserTimer = System.nanoTime() + (long) 2e9;
//                beaconState++;
                driveMixer(0, .5, 0);
                deadShotSays.play(hwMap.appContext, R.raw.a06);
                presserTimer = System.nanoTime() + (long) 1e9;
                beaconState++;
                break;
            case 7:     //continue trying to press the beacon until the color switches to the color of the alliance
                        //or the beacon takes more than 5 seconds to press
//                if(presserTimer < System.nanoTime())
//                    beaconState++;
                if(/*onAllianceColor(isBlue) || */presserTimer < System.nanoTime()){
                    presserSavedTime = System.nanoTime();
                    driveMixer(0, 0, 0);
                    deadShotSays.play(hwMap.appContext, R.raw.a07);
                    beaconState++;
                    resetMotors(true);
                }
                break;
            case 8:
                if(driveStrafe(false, .03, .35)) { beaconState++; }
                break;
            case 9:     //re-align with wall
                if(isBlue){
                    if(RotateIMU(90.5, 1)) beaconState++;
                }
                else{
                    if(RotateIMU(0, 1)) beaconState++;
                }
                break;
            case 10:    //retry all steps from locating the opposing alliance's color to pressing the beacon if
                        //the initial press was unsuccessful
                //if(presserSavedTime > presserTimer)
                //    beaconState = 4;
                 beaconState++;
                break;
            case 11:
                beaconState = 0;
                return true;

        }
        return false;
    }
    public void resetBeaconPresserState(){
        beaconState = 0;
    }

    public double getBatteryVoltage(){
        return RC.h.voltageSensor.get("Motor Controller 1").getVoltage();
    }


}

