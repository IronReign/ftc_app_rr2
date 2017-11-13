package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;


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

    public  double KpDrive = 0.010; //proportional constant multiplier
    private double KiDrive = 0.000; //integral constant multiplier
    private double KdDrive = 0.001; //derivative constant multiplier
    private double driveIMUBasePower = .5;
    private double motorPower = 0;

    DcMotor motorFrontLeft           = null;
    DcMotor motorFrontRight          = null;
    DcMotor motorBackLeft            = null;
    DcMotor motorBackRight           = null;
//    DcMotor motorConveyor            = null; //particle conveyor
//    DcMotor motorLauncher            = null; //flywheel motor
    DcMotor motorLift                = null; //cap ball lift motor
    DcMotor headLamp                 = null; //front white LED string
    DcMotor redLamps                 = null; //side red highlight LED strings
    Servo servoGrip                  = null; //servoGrip for Glyphs and Relics
    Servo servoJewel                 = null; //deploys the arm that knocks off the jewel
    Servo servoTester                = null;
    NormalizedColorSensor colorJewel = null;
    NormalizedRGBA jewelRGB          = null;

//    Servo servoLiftLatch    = null;

    BNO055IMU imu; //Inertial Measurement Unit: Accelerometer and Gyroscope combination sensor
//    Orientation angles; //feedback from the IMU


    private double powerFrontLeft  = 0;
    private double powerFrontRight = 0;
    private double powerBackLeft   = 0;
    private double powerBackRight  = 0;
//    private double powerConveyor   = 0;
    static  int ticksPerRot        = 1680;

//    public ParticleSystem particle = null;
//    public CapTrap cap = null;

    private long flingTimer = 0;
    private int flingSpeed  = 5000; //ticks per second
    private int forwardTPM = 2439; //measurement was for the original 42 tooth driven sprocket, since replaced by a 32 tooth sprocket
    private int strafeTPM = 3145 ; //measurement was for the original 42 tooth driven sprocket, since replaced by a 32 tooth sprocket

    private double poseX;
    private double poseY;
    private double poseHeading; //current heading in degrees. Might be rotated by 90 degrees from imu's heading when strafing
    private double poseHeadingRad; //current heading converted to radians
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

    //scoring objects and related variables
    public PickAndPlace glyphSystem;
    public JewelArm jewel;

    SoundPlayer robotSays = SoundPlayer.getInstance(); //plays audio feedback from the robot controller phone

    private long ticksLeftPrev;
    private long ticksRightPrev;
    private long ticksLeftOffset; //provide a way to offset (effectively reset) the motor encoder readings
    private long ticksRightOffset;
    private double wheelbase; //the width between the wheels

    final protected double flingerRelaxPwr = 0.075;
    final protected double flingerFlingPwr = -1;

    long flingerTimer;

    boolean gripOpen = false;
    public int servoTesterPos = 0;



    private VectorF vuTrans; //vector that calculates the position of the vuforia target relative to the phone (mm)
    private double vuAngle; //angle of the vuforia target from the center of the phone camera (degrees)
    private double vuDepth = 0; //calculated distance from the vuforia target on the z axis (mm)
    private double vuXOffset = 0; //calculated distance from the vuforia target on the x axis (mm)

    public enum MoveMode{
        forward,
        backward,
        left,
        right,
        rotate,
        still;
    }

    protected MoveMode moveMode;



    Orientation imuAngles; //pitch, roll and yaw from the IMU
    protected boolean targetAngleInitialized = false;
    private int beaconState = 0; //switch variable that controls progress through the beacon pressing sequence


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
     * @param angle The vuAngle of the robot
     */
    public Pose(double x, double y, double angle)
    {

        poseX     = x;
        poseY     = y;
        poseHeading = angle;
        poseSpeed = 0;

    }

    /**
     * Creates a base Pose instance at the origin, (_0,_0), with _0 speed and _0 vuAngle.
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

    public void resetTPM(){
        forwardTPM = 2493;
        strafeTPM = 3145;
    }

    /**
     * Initializes motors, servos, lights and sensors from a given hardware map
     *
     * @param ahwMap   Given hardware map
     * @param isBlue   Tells the robot which alliance to initialize for (however initialization is currently alliance independent)
     */
    public void init(HardwareMap ahwMap, boolean isBlue) {
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
        this.motorLift       = this.hwMap.dcMotor.get("motorLift");
        this.headLamp        = this.hwMap.dcMotor.get("headLamp");
        this.redLamps        = this.hwMap.dcMotor.get("redLamps");
        this.servoGrip       = this.hwMap.servo.get("servoGrip");
        this.servoJewel      = this.hwMap.servo.get("servoJewel");
        this.servoTester     = this.hwMap.servo.get("servoTester");
        this.colorJewel      = this.hwMap.get(NormalizedColorSensor.class, "colorJewel");

        jewelRGB = colorJewel.getNormalizedColors();

        //motor configurations

        this.motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motorLift.setDirection(DcMotorSimple.Direction.FORWARD);
        this.motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        moveMode = MoveMode.still;

        this.glyphSystem = new PickAndPlace(motorLift, servoGrip);
        this.jewel = new JewelArm(servoJewel, colorJewel);

        BNO055IMU.Parameters parametersIMU = new BNO055IMU.Parameters();
        parametersIMU.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parametersIMU.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersIMU.loggingEnabled       = true;
        parametersIMU.loggingTag           = "IMU";

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parametersIMU);

        headLampOn();
        redLampOn();
    }

    public void headLampOn(){
        headLamp.setPower(1);
    }
    public void headLampOff(){
        headLamp.setPower(0);
    }
    public void redLampOn(){
        redLamps.setPower(1);
    }
    public void redLampOff(){
        redLamps.setPower(0);
    }

    /**
     * Moves the mecanum platform under PID control applied to the rotation of the robot. This version can either drive forwards/backwards or strafe.
     *
     * @param Kp   proportional constant multiplier
     * @param Ki   integral constant multiplier
     * @param Kd   derivative constant multiplier
     * @param pwr  base motor power before correction is applied
     * @param currentAngle   current angle of the robot in the coordinate system of the sensor that provides it- should be updated every cycle
     * @param targetAngle   the target angle of the robot in the coordinate system of the sensor that provides the current angle
     * @param strafe   if true, the robot will drive left/right. if false, the robot will drive forwards/backwards.
     */
    public void movePID(double Kp, double Ki, double Kd, double pwr, double currentAngle, double targetAngle, boolean strafe) {
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
        if(strafe) driveMixer(0, pwr, correction);
        else driveMixer(pwr, 0, correction);
    }

    public void stopAll(){
        glyphSystem.stopLift();
        driveMixer(0, 0, 0);
    }

    /**
     * Moves the mecanum platform under PID control applied to the rotation of the robot. This version can drive forwards/backwards and strafe simultaneously.
     *
     * @param Kp   proportional constant multiplier
     * @param Ki   integral constant multiplier
     * @param Kd   derivative constant multiplier
     * @param pwrFwd  base forwards/backwards motor power before correction is applied
     * @param pwrStf  base left/right motor power before correction is applied
     * @param currentAngle   current angle of the robot in the coordinate system of the sensor that provides it- should be updated every cycle
     * @param targetAngle   the target angle of the robot in the coordinate system of the sensor that provides the current angle
     */
    public void movePIDMixer(double Kp, double Ki, double Kd, double pwrFwd, double pwrStf, double currentAngle, double targetAngle) {
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
        driveMixer(pwrFwd, pwrStf, correction);
    }


    public void driveIMU(double Kp, double Ki, double Kd, double pwr, double targetAngle, boolean strafe){
        movePID(Kp, Ki, Kd, pwr, poseHeading, targetAngle, strafe);
    }

    public void driveIMUMixer(double Kp, double Ki, double Kd, double pwrFwd, double pwrStf, double targetAngle){
        movePIDMixer(Kp, Ki, Kd, pwrFwd, pwrStf, poseHeading, targetAngle);
    }

    public boolean driveIMUDistance(double Kp, double pwr, double targetAngle, boolean forwardOrLeft, double targetMeters, boolean strafe){
        if(!forwardOrLeft){
            moveMode = moveMode.backward;
            targetMeters = -targetMeters;
            pwr = -pwr;
        }
        else moveMode = moveMode.forward;
        long targetPos;
        if(strafe) targetPos = (long) targetMeters * strafeTPM;
        else targetPos = (long)(targetMeters * forwardTPM);
        if(Math.abs(targetPos) > Math.abs(getAverageAbsTicks())){//we've not arrived yet
            driveIMU(Kp, KiDrive, KdDrive, pwr, targetAngle, strafe);
            return false;
        }
        else { //destination achieved
            driveMixer(0, 0, 0);
            return true;
        }
    }

    public boolean rotateIMU(double targetAngle, double maxTime){ //uses default pose PID constants and has end conditions
        if(!turnTimerInit){
            turnTimer = System.nanoTime() + (long)(maxTime * (long) 1e9);
            turnTimerInit = true;
        }
        driveIMU(KpDrive, KiDrive, KdDrive, 0, targetAngle, false); //if the robot turns within a threshold of the target
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

    public void raiseGlyph(){
        motorLift.setPower(.5);
    }
    public void lowerGlyph(){
        motorLift.setPower(-.5);
    }
    public void stopGlyph(){
        motorLift.setPower(0);
    }





    public void maintainHeading(boolean buttonState){
        if(buttonState) {
            if (!maintainHeadingInit) {
                poseSavedHeading = poseHeading;
                maintainHeadingInit = true;}
            driveIMU(KpDrive, KiDrive, KdDrive, 0, poseSavedHeading, false);
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
        initialized = false; //triggers recalc of heading offset at next IMU update cycle
    }

    public void servoTester(boolean bigUp, boolean smallUp, boolean smallDown, boolean bigDown){
        if(bigUp){
            servoTesterPos += 25;
        }
        if(smallUp){
            servoTesterPos += 100;
        }
        if(smallDown){
            servoTesterPos -= 25;
        }
        if(bigDown){
            servoTesterPos -= 100;
        }
        servoTester.setPosition(servoNormalize(servoTesterPos));
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

        powerFrontLeft += rotate;
        powerBackLeft += rotate;
        powerFrontRight -= rotate;
        powerBackRight -= rotate;

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


    public boolean driveForward(boolean forward, double targetMeters, double power){
        if(!forward){
            moveMode = moveMode.backward;
            targetMeters = -targetMeters;
            power = -power;
        }
        else moveMode = moveMode.forward;

        long targetPos = (long)(targetMeters * forwardTPM);
        if(Math.abs(targetPos) > Math.abs(getAverageTicks())){//we've not arrived yet
            driveMixer(power, 0, 0);
            return false;
        }
        else { //destination achieved
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

        long targetPos = (long)(targetMeters * strafeTPM);
        if(Math.abs(targetPos) > Math.abs(getAverageAbsTicks())){
            driveMixer(0, power, 0);
            return false;
        }
        else {
            driveMixer(0, 0, 0);
            return true;
        }
    }

//    boolean rotateRelative(boolean clockwise, double targetAngle, double power){
//        moveMode = moveMode.rotate;
//        if(!clockwise){
//            targetAngle = -targetAngle;
//            power = -power;
//        }
//        if(!targetAngleInitialized) { targetAngle = targetAngle + angles.firstAngle; targetAngleInitialized = true; }
//        if(Math.abs(targetAngle) > Math.abs(angles.firstAngle)){
//            driveMixer(0, 0, power);
//            return false;
//        }
//        else {
//            driveMixer(0, 0, 0);
//            return true;
//        }
//    }


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
        initialized = false; //trigger recalc of offset on next update
    }

    /**
     * Set the absolute pitch of the robot _0-360 degrees
     * @param posePitch
     */
    public void setPosePitch(double posePitch) {
        this.posePitch = posePitch;
        initialized = false; //trigger recalc of offset on next update
    }

    /**
     * Set the absolute roll of the robot _0-360 degrees
     * @param poseRoll
     */
    public void setPoseRoll(double poseRoll) {
        this.poseRoll = poseRoll;
        initialized = false; //trigger recalc of offset on next update
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

    public long getForwardTPM() {
        return forwardTPM;
    }

    public void setForwardTPM(long forwardTPM) { this.forwardTPM = (int) forwardTPM; }

    public long getStrafeTPM() {
        return strafeTPM;
    }

    public void setStrafeTPM(long strafeTPM) { this.strafeTPM = (int) strafeTPM; }

    public void setTicksPerMeterLeft(long TPM_Strafe) {
        this.strafeTPM = (int)TPM_Strafe;
    }

    public void updateSensors(){
        update(imu, 0, 0);
    }

    public boolean doesJewelMatch(boolean isBlue){
        if(isBlue){
            return (jewelRGB.blue > 128);
        }
        return (jewelRGB.red > 128);
    }

    /**
     * update the current location of the robot. This implementation gets heading and orientation
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

    public void update(BNO055IMU imu, long ticksLeft, long ticksRight){
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
                displacement = (getAverageTicks() - displacementPrev) * forwardTPM;
                odometer += Math.abs(displacement);
                poseHeadingRad = Math.toRadians(poseHeading);
                break;
            case left:
                displacement = (getAverageAbsTicks() - displacementPrev) * strafeTPM; //todo: there might be a problem when switching between driving forward and strafing - displacementPrev may need to be updated the first time in to use the correct avgTicks calculation
                poseHeadingRad = Math.toRadians(poseHeading)- Math.PI/2; //actual heading is rotated 90 degrees counterclockwise

                break;
            case right:
                displacement = (getAverageAbsTicks() - displacementPrev) * strafeTPM; //todo: there might be a problem when switching between driving forward and strafing - displacementPrev may need to be updated the first time in to use the correct avgTicks calculation
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


    public static double servoNormalize(int pulse){
        double normalized = (double)pulse;
        return (normalized - 750.0) / 1500.0; //convert mr servo controller pulse width to double on _0 - 1 scale
    }

    public double driveToBeacon(VuforiaTrackableDefaultListener beacon, boolean isBlue, int beaconConfig, double bufferDistance, double maxSpeed, boolean turnOnly, boolean offset) {

        //double vuDepth = 0;
        double pwr = 0;

        if (beacon.getPose() != null) {
            vuTrans = beacon.getRawPose().getTranslation();

            //todo - add a new transform that will shift our target left or right depending on beacon analysis

            if(offset){vuAngle = Math.toDegrees(Math.atan2(vuTrans.get(0) + getBeaconOffset(isBlue, beaconConfig), vuTrans.get(2)));}
            else vuAngle = Math.toDegrees(Math.atan2(vuTrans.get(0), vuTrans.get(2)));
            vuDepth = vuTrans.get(2);

            if (turnOnly)
                pwr = 0; //(vuDepth - bufferDistance/1200.0);
            else
                // this is a very simple proportional on the distance to target - todo - convert to PID control
                pwr = clampDouble(-maxSpeed, maxSpeed, ((bufferDistance - vuDepth)/1200.0));//but this should be equivalent
            Log.i("Beacon Angle", String.valueOf(vuAngle));
            movePID(KpDrive, KiDrive, KdDrive, pwr, -vuAngle, 0, false);

        } else { //disable motors if given target not visible
            vuDepth = 0;
            driveMixer(0,0,0);
        }//else

    return vuDepth; // 0 indicates there was no good vuforia pose - target likely not visible
    }//driveToBeacon

    public double strafeBeacon(VuforiaTrackableDefaultListener beacon, double offsetDistance, double pwrMax, double iWishForThisToBeOurHeading) {

        //double vuDepth = 0;
        double pwr = 0;

        if (beacon.getPose() != null) {
            vuTrans = beacon.getRawPose().getTranslation();

            //todo - add a new transform that will shift our target left or right depending on beacon analysis

            vuAngle = Math.toDegrees(Math.atan2(vuTrans.get(0), vuTrans.get(2)));
            vuXOffset = vuTrans.get(0);

                            // this is a very simple proportional on the distance to target - todo - convert to PID control
            pwr = clampDouble(-pwrMax, pwrMax, ((vuXOffset - offsetDistance)/1200.0));//but this should be equivalent
            Log.i("Beacon Angle", String.valueOf(vuAngle));
            driveIMU(KpDrive, KiDrive, KdDrive, pwr, iWishForThisToBeOurHeading, true);

        } else { //disable motors if given target not visible
            vuDepth = 0;
            driveMixer(0,0,0);
        }//else

        return vuDepth - offsetDistance; // 0 indicates there was no good vuforia pose - target likely not visible
    }//driveToBeacon

    public double getBeaconOffset(boolean isBlue, int beaconConfig){
        double offset;
        if((isBlue && beaconConfig == 1) || (!isBlue && beaconConfig == 2)){
            offset = -80;
        }
        else {
            offset = 80;
        }
        return offset;
    }

    public double getVuAngle(){
        return vuAngle;
    }
    public double getVuDepth(){
        return vuDepth;
    }
    public double getVuXOffset(){
        return vuXOffset;
    }


    public double getBatteryVoltage(){
        return RC.h.voltageSensor.get("Motor Controller 1").getVoltage();
    }


}

