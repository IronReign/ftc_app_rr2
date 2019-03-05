package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/**
 * The Pose class stores the current real world position/orientation: <b>position</b>, <b>heading</b>,
 * and <b>speed</b> of the robot.
 *
 * This class should be a point of reference for any navigation classes that want to know current
 * orientation and location of the robot.  The update method must be called regularly, it
 * monitors and integrates data from the orientation (IMU) and odometry (motor encoder) sensors.
 * @author Abhijit Bhattaru
 * @version 3.0
 * @since 2018-11-02
 */

public class PoseBigWheel
{

    HardwareMap hwMap;

    //motors

    PIDController drivePID = new PIDController(0, 0, 0);

    public double kpDrive = 0.010; //proportional constant multiplier
    public double kiDrive = 0.000; //integral constant multiplier
    public double kdDrive = 0.001; //derivative constant multiplier


    DcMotor driveLeft = null;
    DcMotor driveRight = null;
    DcMotor elbowLeft = null;
    DcMotor elbowRight = null;
    DcMotor extendABobLeft = null;
    DcMotor extendABobRight = null;
    DcMotor supermanMotor = null;
    Servo intakeRight = null;
    Servo intakeLeft = null;
    Servo hook = null;
    Servo intakeGate = null;
    Servo blinkin = null;



    //subsystems

    Collector collector = null;
    Superman superman;
    LEDSystem ledSystem;

    BNO055IMU imu; //Inertial Measurement Unit: Accelerometer and Gyroscope combination sensor
//    Orientation angles; //feedback from the IMU
    DistanceSensor distForward;
    DistanceSensor distLeft;
    DistanceSensor distRight;
    Rev2mDistanceSensor sensorTimeOfFlight;


    private double powerLeft = 0;
    private double powerRight = 0;

    private boolean isIntakeOn = false;
    private double intakeOn = 1;
    private double intakeOff = .5;
    static  int ticksPerRot        = 1680;

    private long flingTimer = 0;
    private int flingSpeed  = 5000; //ticks per second
    private int forwardTPM = (int) (1078); //measurement was for the original 42 tooth driven sprocket, since replaced by a 32 tooth sprocket

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
    int balanceState = 1;

    long timeHook = 0;

    SoundPlayer robotSays = SoundPlayer.getInstance(); //plays audio feedback from the robot controller phone

    private long ticksLeftPrev;
    private long ticksRightPrev;
    private long ticksLeftOffset; //provide a way to offset (effectively reset) the motor encoder readings
    private long ticksRightOffset;
    private double wheelbase; //the width between the wheels

    public int servoTesterPos = 1600;

    public double autonomousIMUOffset = 0;

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

    public enum Articulation{ //serves as a desired robot articulation which may include related complex movements of the elbow, lift and superman
        inprogress, //currently in progress to a final articulation
        manual, //target positions are all being manually overridden
        driving, //optimized for driving - elbow opened a bit, lift extended a bit - shifts weight toward drive wheels for better turn and drive traction
        hanging, //auton initial hang at the beginning of a match
        deploying, //auton unfolding after initial hang - should only be called from the hanging position during auton - ends when wheels should be on the ground, including superman, and pressure is off of the hook
        deployed, //auton settled on ground - involves retracting the hook, moving forward a bit to clear lander and then lowering superman to driving position
        cratered, //auton arm extended over the crater - this might end up being the same as preIntake
        preIntake, //teleop mostly - collector retracted and open to almost ground level
        intake,     //teleop mostly - collector extended low, intaking - intake pushing on ground, extension overrideable
        deposit, //teleop mostly - transition from intake to deposit - retract collector to low position waiting on completion, close elbow to deposit position, superman up to deposit position, extend collector to deposit position
        latchApproach, //teleop endgame - driving approach for latching, expected safe to be called from manual, driving, deposit - set collector elbow for drive balance, extended to max and superman up,
        latchPrep, //teleop endgame - make sure hook is open, set drivespeed slow, extend lift to max, finalize elbow angle for latch, elbow overrideable
        latchSet, //teleop endgame - close the latch
        latchHang; //teleop endgame - close collector elbow to final position, set locks if implemented
    }

    public Articulation getArticulation() {
        return articulation;
    }

    protected Articulation articulation = Articulation.manual;
    double articulationTimer = 0;

    Orientation imuAngles; //pitch, roll and yaw from the IMU
    protected boolean targetAngleInitialized = false;
    private int beaconState = 0; //switch variable that controls progress through the beacon pressing sequence

    public boolean isAutonSingleStep() {
        return autonSingleStep;
    }

    public void setAutonSingleStep(boolean autonSingleStep) {
        this.autonSingleStep = autonSingleStep;
    }

    boolean autonSingleStep = false; //single step through auton deploying stages to facilitate testing and demos



    //////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////
    ////                                                                                  ////
    ////                                 Constructors                                     ////
    ////                                                                                  ////
    //////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////


    /**
     * Create a Pose instance that stores all real world position/orientation elements:
     * <var>x</var>, <var>y</var>, <var>heading</var>, and <var>speed</var>.
     *
     * @param x     The position relative to the x axis of the field
     * @param y     The position relative to the y axis of the field
     * @param heading The heading of the robot
     * @param speed The speed of the robot
     */
    public PoseBigWheel(double x, double y, double heading, double speed)
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
    public PoseBigWheel(double x, double y, double angle)
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
    public PoseBigWheel()
    {

        poseX     = 0;
        poseY     = 0;
        poseHeading = 0;
        poseSpeed = 0;
        posePitch=0;
        poseRoll=0;

    }


    //////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////
    ////                                                                                  ////
    ////                                 Init/Update                                      ////
    ////                                                                                  ////
    //////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////



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


        this.driveLeft = this.hwMap.dcMotor.get("driveLeft");
        this.driveRight = this.hwMap.dcMotor.get("driveRight");
        this.elbowLeft = this.hwMap.dcMotor.get("elbowLeft");
        this.elbowRight = this.hwMap.dcMotor.get("elbowRight");
        this.extendABobLeft = this.hwMap.dcMotor.get("liftLeft");
        this.extendABobRight = this.hwMap.dcMotor.get("liftRight");
        this.intakeRight = this.hwMap.servo.get("intakeRight");
        this.intakeLeft = this.hwMap.servo.get("intakeLeft");
        this.supermanMotor = this.hwMap.dcMotor.get("supermanMotor");
        this.hook = this.hwMap.servo.get("hook");
        this.intakeGate = this.hwMap.servo.get("intakeGate");
        this.blinkin = this.hwMap.servo.get("blinkin");

        driveRight.setDirection(DcMotorSimple.Direction.REVERSE);
        driveLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        collector = new Collector(elbowLeft, elbowRight, extendABobLeft, extendABobRight, intakeRight, intakeLeft, hook, intakeGate);
        collector.setElbowPwr(.5);
        superman = new Superman(supermanMotor);
        ledSystem = new LEDSystem(blinkin);

        isIntakeOn = false;


//        this.driveRight.setDirection(DcMotorSimple.Direction.REVERSE);

        moveMode = MoveMode.still;

        BNO055IMU.Parameters parametersIMU = new BNO055IMU.Parameters();
        parametersIMU.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parametersIMU.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersIMU.loggingEnabled = true;
        parametersIMU.loggingTag = "IMU";

        BNO055IMU.Parameters parametersIMULift = new BNO055IMU.Parameters();
        parametersIMULift.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parametersIMULift.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersIMULift.loggingEnabled = true;
        parametersIMULift.loggingTag = "IMULift";


        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parametersIMU);

        distForward = hwMap.get(DistanceSensor.class, "distForward");
        distRight = hwMap.get(DistanceSensor.class, "distRight");
        distLeft = hwMap.get(DistanceSensor.class, "distLeft");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)distForward;

    }



    public void resetIMU(){
        BNO055IMU.Parameters parametersIMU = new BNO055IMU.Parameters();
        parametersIMU.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parametersIMU.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersIMU.loggingEnabled = true;
        parametersIMU.loggingTag = "IMU";

        imu.initialize(parametersIMU);
    }

    public void resetEncoders(){
        superman.resetEncoders();
        collector.resetEncoders();
    }

    /**
     * update the current location of the robot. This implementation gets heading and orientation
     * from the Bosch BNO055 IMU and assumes a simple differential steer robot with left and right motor
     * encoders. also updates the positions of robot subsystems; make sure to add each subsystem's update class as more are implemented.
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

        if(posePitch>40){
            superman.restart(.6);
        }

        articulate(articulation); //call the most recently requested articulation
        collector.update();
        superman.update();




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

    public void updateSensors(){
        update(imu, 0, 0);
    }



    //////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////
    ////                                                                                  ////
    ////                               Movement/Actions                                   ////
    ////                                                                                  ////
    //////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////


    /**
     *
     * @param forward
     * @param targetMeters
     * @param power
     * @return
     */
    public boolean driveForward(boolean forward, double targetMeters, double power){
        if(!forward){
            moveMode = moveMode.backward;
            targetMeters = -targetMeters;
            power = -power;
        }
        else moveMode = moveMode.forward;

        long targetPos = (long)(targetMeters * forwardTPM);
        if(Math.abs(targetPos) > Math.abs(getAverageTicks())){//we've not arrived yet
            driveMixerTank(power, 0);
            return false;
        }
        else { //destination achieved
            driveMixerTank(power, 0);
            return true;
        }
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
     */
    public void movePID(double Kp, double Ki, double Kd, double pwr, double currentAngle, double targetAngle) {

        //initialization of the PID calculator's output range, target value and multipliers
        drivePID.setOutputRange(-.5,.5);
        drivePID.setPID(Kp, Ki, Kd);
        drivePID.setSetpoint(targetAngle);
        drivePID.enable();

        //initialization of the PID calculator's input range and current value
        drivePID.setInputRange(0, 360);
        drivePID.setContinuous();
        drivePID.setInput(currentAngle);

        //calculates the correction to supply to the motor
        double correction = drivePID.performPID();

        //performs the drive with the correction applied
        driveMixerTank(pwr, correction);
    }

    /**
     * Stops all motors on the robot
     */
    public void stopAll(){
        driveMixerTank(0,0);
    }


    /**
     * Drive forwards for a set power while maintaining an IMU heading using PID
     * @param Kp proportional multiplier for PID
     * @param Ki integral multiplier for PID
     * @param Kd derivative proportional for PID
     * @param pwr set the forward power
     * @param targetAngle the heading the robot will try to maintain while driving
    */
    public void driveIMU(double Kp, double Ki, double Kd, double pwr, double targetAngle){
        movePID(Kp, Ki, Kd, pwr, poseHeading, targetAngle);
    }



    /**
     * Drive with a set power for a set distance while maintaining an IMU heading using PID
     * @param Kp proportional multiplier for PID
     * @param pwr set the forward power
     * @param targetAngle the heading the robot will try to maintain while driving
     * @param forwardOrLeft is the robot driving in the forwards/left (positive) directions or backwards/right (negative) directions
     * @param targetMeters the target distance (in meters)
     */
    public boolean driveIMUDistance(double Kp, double pwr, double targetAngle, boolean forwardOrLeft, double targetMeters){

        //set what direction the robot is supposed to be moving in for the purpose of the field position calculator
        if(!forwardOrLeft){
            moveMode = moveMode.backward;
            targetMeters = -targetMeters;
            pwr = -pwr;
        }
        else moveMode = moveMode.forward;

        //calculates the target position of the drive motors
        long targetPos;
        targetPos = (long)(targetMeters * forwardTPM);

        //if this statement is true, then the robot has not achieved its target position
        if(Math.abs(targetPos) < Math.abs(getAverageAbsTicks())){
            driveIMU(Kp, kiDrive, kdDrive, pwr, targetAngle);
            return false;
        }

        //destination achieved
        else {
            driveMixerTank(0, 0);
            return true;
        }
    }


    /**
     * Rotate to a specific heading with a time cutoff in case the robot gets stuck and cant complete the turn otherwise
     * @param targetAngle the heading the robot will attempt to turn to
     * @param maxTime the maximum amount of time allowed to pass before the sequence ends
     */
    public boolean rotateIMU(double targetAngle, double maxTime){
        if(!turnTimerInit){ //intiate the timer that the robot will use to cut of the sequence if it takes too long; only happens on the first cycle
            turnTimer = System.nanoTime() + (long)(maxTime * (long) 1e9);
            turnTimerInit = true;
        }
        driveIMU(kpDrive, kiDrive, kdDrive, 0, targetAngle); //check to see if the robot turns within a threshold of the target
        if(Math.abs(poseHeading - (targetAngle + autonomousIMUOffset)) < minTurnError) {
            turnTimerInit = false;
            driveMixerTank(0,0);
            return true;
        }
        if(turnTimer < System.nanoTime()){ //check to see if the robot takes too long to turn within a threshold of the target (e.g. it gets stuck)
            turnTimerInit = false;
            driveMixerTank(0,0);
            return true;
        }
        return false;
    }


    /**
     * the maintain heading function used in demo: holds the heading read on initial button press
     * @param buttonState the active state of the button; if true, hold the current position. if false, do nothing
     */
    public void maintainHeading(boolean buttonState){

        //if the button is currently down, maintain the set heading
        if(buttonState) {
            //if this is the first time the button has been down, then save the heading that the robot will hold at and set a variable to tell that the heading has been saved
            if (!maintainHeadingInit) {
                poseSavedHeading = poseHeading;
                maintainHeadingInit = true;
            }
            //hold the saved heading with PID
            driveIMU(kpDrive, kiDrive, kdDrive, 0, poseSavedHeading);
        }

        //if the button is not down, set to make sure the correct heading will be saved on the next button press
        if(!buttonState){
            maintainHeadingInit = false;
        }
    }


    /**
     * a method written to test servos by plugging them into a designated servo tester port on the REV module
     * designed to work best with debounced gamepad buttons
     * @param largeUp if true, increase PWM being sent to the servo tester by a large amount
     * @param smallUp if true, increase PWM being sent to the servo tester by a small amount
     * @param smallDown if true, decrease PWM being sent to the servo tester by a small amount
     * @param largeDown if true, decrease PWM being sent to the servo tester by a large amount
     */
    public void servoTester(boolean largeUp, boolean smallUp, boolean smallDown, boolean largeDown){
        //check to see if the PWM value being sent to the servo should be altered
        if(largeUp){
            servoTesterPos += 100;
        }
        if(smallUp){
            servoTesterPos += 25;
        }
        if(smallDown){
            servoTesterPos -= 25;
        }
        if(largeDown){
            servoTesterPos -= 100;
        }

        //send the PWM value to the servo regardless of if it is altered or not
    }

    public boolean autoBalance (){

        switch(balanceState){
            case 1:
                 //posePitch;
                driveMixerTank(.2, 0);
                if(posePitch<=0){
                   balanceState++;
                }
                break;
            case 2:
                if(posePitch<-4){
                    driveMixerTank(-.1,0);
                }else{
                    balanceState++;
                }
                break;
            case 3:
                driveMixerTank(0,0);
                balanceState=1;
                return true;
        }
        return false;
    }


    //////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////
    ////                                                                                  ////
    ////                        Articulations                                             ////
    ////                                                                                  ////
    //////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////

   /*
       public enum Articulation{ //serves as a desired robot articulation which may include related complex movements of the elbow, lift and superman
        manual, //target positions are all being manually overridden
        driving, //optimized for driving - elbow opened a bit, lift extended a bit - shifts weight toward drive wheels for better turn and drive traction
        hanging, //auton initial hang at the beginning of a match
        deploying, //auton unfolding after initial hang - should only be called from the hanging position during auton - ends when wheels should be on the ground, including superman, and pressure is off of the hook
        deployed, //auton settled on ground - involves retracting the hook, moving forward a bit to clear lander and then lowering superman to driving position
        cratered, //auton arm extended over the crater - this might end up being the same as preIntake
        preIntake, //teleop mostly - collector retracted and open to almost ground level
        intake,     //teleop mostly - collector extended low, intaking - intake pushing on ground, extension overrideable
        deposit, //teleop mostly - transition from intake to deposit - retract collector to low position waiting on completion, close elbow to deposit position, superman up to deposit position, extend collector to deposit position
        latchApproach, //teleop endgame - driving approach for latching, expected safe to be called from manual, driving, deposit - set collector elbow for drive balance, extended to max and superman up,
        latchPrep, //teleop endgame - make sure hook is open, set drivespeed slow, extend lift to max, finalize elbow angle for latch, elbow overrideable
        latchSet, //teleop endgame - close the latch
        latchHang; //teleop endgame - close collector elbow to final position, set locks if implemented
    }
    */
   double miniTimer;
   int miniState = 0;

   public boolean articulate(Articulation target, boolean setAndForget){
        articulate(target);
        return true;
   }

   public boolean articulate(Articulation target) {
       articulation = target; //store the most recent explict articulation request as our target, allows us to keep calling incomplete multi-step transitions
       if (target == Articulation.manual) {
           miniState = 0; //reset ministate - it should only be used in the context of a multi-step transition, so safe to reset it here
       }

       switch (articulation) {
           case manual:
               break; //do nothing here - likely we are directly overriding articulations in game
           case driving:
                if (goToSafeDrive()) return true;
               break;
           case hanging: //todo: fixup comments for deploy actions - moved stuff around
               //auton initial hang at the beginning of a match
                collector.setExtendABobTargetPos(0);
                //collector.hookOn();
                collector.setElbowTargetPos(10,1);
                return true;
               break;
           case deploying:
               //auton unfolding after initial hang - should only be called from the hanging position during auton
               // ends when wheels should be on the ground, including superman, and pressure is off of the hook
               collector.extendToMid(1, 15);
               if(collector.setElbowTargetPos(collector.pos_autonPrelatch, .5)) {
                   superman.setTargetPosition(superman.pos_prelatch, 1);
                   if (driveForward(false, .1, .2)) {
                       driveMixerTank(0,0);
                       //if (superman.setTargetPosition(superman.pos_prelatch, 1)) //lower superman so it's ready to support robot, but not pushing up on hook
                       //{
                           miniState = 0; //reset nested state counter for next use
                           if (!isAutonSingleStep()) articulation = Articulation.deployed; //auto advance to next stage
                           else articulation = Articulation.manual;
                           return true; // signal advance to the deployed stage

                       //}
                       //break;
                   }
                   break;
               }
               break;

               //wait until on floor as indicated by time or imu angle or superman position or distance sensor - whatever is reliable enough
               //for now we wait on elapsed time to complete sequence
               /*
               articulationTimer = futureTime(2); //setup wait for completion. todo: change this to position based auto advancement

               */
           case deployed:
               //auton settled on ground - involves retracting the hook,
               // moving forward a bit to clear lander and then
               // lowering superman to driving position
               if (System.nanoTime() >= articulationTimer) {
                   switch (miniState) {
                       case 0:  //push lightly into lander to relieve pressure on hook
                           collector.hookOff(); //retract hook
                           miniTimer=futureTime(1);
                           miniState++;
                           break;
                           //if (driveForward(false, .2, .7)) miniState++;
                           //miniTimer = futureTime(1); //setup wait for completion

                       case 1:  //retract lander hook
                           if (System.nanoTime() >= miniTimer) {
                               if (rotateIMU(350, 1)) { //this turn is needed because hook doesn't clear entirely
                                   resetMotors(true);
                                   miniTimer = futureTime(1); //setup wait for completion
                                   miniState++;
                               }
                           }
                           break;

                       case 2://pull away from lander
                           if (System.nanoTime() >= miniTimer) {
                               if(driveForward(true, .25, .4)) {
                                   driveMixerTank(0, 0); //stop drive motors
                                   //miniTimer = futureTime(1); //setup wait for completion
                                   miniState++;
                               }
                           }
                           break;
                       case 3:  //automatically transition to driving articulation
                           if (System.nanoTime() >= miniTimer) {
                               miniState = 0; //just being a good citizen for next user of miniState
                               articulation = Articulation.driving; //force transition to driving articulation
                               return true; //force transition to driving articulation
                           }

                           break;
                   }

               }
                   else break;
           case cratered:
               break;
           case preIntake:
               goToPreIntake();
               break;
           case intake:
               collector.closeGate();
               goToIntake();
               break;
           case deposit:
               //goToDeposit();
               switch (miniState) { //todo: this needs to be more ministages - need an interim aggressive close of the elbow followed by superman, followed by opening the elbow up again, all before the extendMax
                   case 0: //set basic speeds and start closing elbow to manage COG
                       collector.restart(.25, 1);
                       superman.restart(.75);
                       if (collector.setElbowTargetPos(collector.pos_PartialDeposit,1))
                       miniState++; //close elbow as fast as possible and hold state until completion
                       break;
                   case 1: //rise up
                       collector.extendToMid(1,15);
                       if (superman.setTargetPosition(superman.pos_DepositPartial, 1)) miniState++; //start going really fast to interim position
                       break;
                   case 2:
                       //if (collector.extendToMid(1,15))
                       miniState++;
                       break;
                   case 3:
                       if (collector.setElbowTargetPos(collector.pos_Deposit, 1)) {  //elbow back out to deposit position
                           miniState++;
                       }
                       break;
                   case 4:
                       superman.setTargetPosition(superman.pos_Deposit, .4); //slow on remaining rotation to minimize overshoot
                       driveForward(true,.4, 1); //drive toward lander - helps pre-position  for deposit and slightly counters the robots tendency to over-rotate toward the lander because of all of the other moves
                       if (collector.extendToMax(1,15)) {
                           collector.openGate(); //experimental - auto open gate requires that we are on-target side to side and in depth - not really ready for this but wanting to try it out
                           miniState = 0; //just being a good citizen for next user of miniState
                           articulation = Articulation.manual; //force end of articulation by switching to manual
                           return true;
                       }
                       break;
               }

               break;
           case latchApproach://teleop endgame - driving approach for latching,
               // expected safe to be called from manual, driving, deposit -
               // set collector elbow for drive balance, extended to mid and superman up
               switch (miniState) {
                   case 0: //set superman
                       collector.restart(.40, .5);
                       superman.restart(.4);
                       superman.setTargetPosition(superman.pos_prelatch-150);
                       if(superman.nearTarget()) miniState++;
                       break;
                   case 1: //set collector
                       collector.restart(.40, .5);
                       superman.restart(.1);
                       superman.setTargetPosition(superman.pos_prelatch-150);
                       collector.setElbowTargetPos(collector.pos_prelatch);
                       collector.extendToMid(1, 15);
                       if(collector.nearTarget()) {
                           miniState = 0;
                           articulation = Articulation.manual;
                           return true;
                       }
                       break;
               }
               break;
           case latchPrep://teleop endgame - make sure hook is open,
               // set drivespeed slow, extend lift to mid, finalize elbow angle for latch, elbow overrideable
               switch (miniState) {
                   case 0:
                       superman.restart(.60);
                       collector.restart(.30, .75);
                       superman.setTargetPosition(superman.pos_latched);
                       if(superman.getCurrentPosition()>collector.pos_PartialDeposit)
                           collector.setElbowTargetPos(collector.pos_latched);
                       if (superman.nearTarget()){
                           //beep();
                           miniState++;
               }
                       break;
                   case 1:
                       superman.restart(.60);
                       collector.restart(.30,.75);
                       collector.setElbowTargetPos(collector.pos_latched);
                       if(collector.nearTarget()) {
                           miniState = 0;
                           articulation = Articulation.manual;
                           return true;
                       }
                       break;
               }
               break;
           case latchSet:
               collector.restart(.40, .5);
               superman.restart(.75);
               if(superman.setTargetPosition(superman.pos_postlatch, 1))
                   collector.setElbowTargetPos(collector.pos_postlatch);
               articulation = Articulation.manual;
               return true;
               //break;
           case latchHang:
               break;
           default:
               return false;

       }
       return false;
   }


    //////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////
    ////                                                                                  ////
    ////                        Superman/Elbow control functions                          ////
    ////                                                                                  ////
    //////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////

//todo these need to be tested - those that are used in articulate() have probably been fixed up by now

    public boolean Deploy(){
       articulate(Articulation.deploying);

       return true;
    }
    public boolean goToPreLatch(){
        collector.restart(.40, .5);
        superman.restart(.75);
        superman.setTargetPosition(superman.pos_prelatch);
        collector.setElbowTargetPos(collector.pos_prelatch);
        collector.extendToMid(1, 15);
        if(collector.nearTarget() && superman.nearTarget())  return true;
        else return false;
    }

    public boolean goToLatchSuperman(){
        superman.restart(.60);
        collector.restart(.30,.75);
        superman.setTargetPosition(superman.pos_latched);
        if(superman.nearTarget())  return true;
        else return false;
    }

    public boolean goToLatchElbow(){
        superman.restart(.60);
        collector.restart(.30,.75);
        collector.setElbowTargetPos(collector.pos_latched);
        if(collector.nearTarget())  return true;
        else return false;
    }

    public boolean goToPostLatch(){
        collector.restart(.40, .5);
        superman.restart(.75);
        if(superman.setTargetPosition(superman.pos_postlatch, 1))
            collector.setElbowTargetPos(collector.pos_postlatch);
        else
            return false;

        if(collector.nearTarget() && superman.nearTarget())  return true;
        else return false;
    }

    public boolean goToPreIntake(){  //should usually be called from deposit position
                                    //todo: needs time to retract lift before moving elbow - slow elbow speed may not be good enough
        collector.restart(1, 1);
        superman.restart(1);
        superman.setTargetPosition(superman.pos_Intake);
        collector.setElbowTargetPos(collector.pos_preIntake);
        collector.extendToLow(1,15);

        if(collector.nearTarget() && superman.nearTarget())  return true;
        else return false;
    }

    public boolean goToIntake(){
        collector.restart(.40, .5);
        superman.restart(.75);
        superman.setTargetPosition(superman.pos_Intake);
        collector.setElbowTargetPos(collector.pos_Intake);
        collector.extendToLow(1,15);

        if(collector.nearTarget() && superman.nearTarget())  return true;
        else return false;
    }

    public boolean goToSafeDrive(){
        collector.restart(.40, .5);
        superman.restart(.75);
        superman.setTargetPosition(superman.pos_driving);
        collector.setElbowTargetPos(collector.pos_SafeDrive);
        collector.extendToLow(); //set arm extension to preset for intake which helps move COG over drive wheels a bit
        collector.closeGate();

        if(collector.nearTarget() && superman.nearTarget())  return true;
        else return false;
    }

    public boolean goToDeposit(){
        collector.restart(1, 1);
        superman.restart(.25);
        superman.setTargetPosition(superman.pos_Deposit);
        collector.setElbowTargetPos(collector.pos_Deposit);
        collector.extendToMax(.25,15);

        if(collector.nearTarget() && superman.nearTarget())  return true;
        else return false;
    }



    //////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////
    ////                                                                                  ////
    ////                         Drive Platform Mixing Methods                            ////
    ////                                                                                  ////
    //////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////



    /**
     * drive method for a mecanum drive
     * @param forward sets how much power will be provided in the forwards direction
     * @param rotate sets how much power will be provided to clockwise rotation
     */
    public void driveMixerTank(double forward, double rotate){

        //reset the power of all motors
        powerRight = 0;
        powerLeft = 0;

        //set power in the forward direction
        powerLeft = forward;
        powerRight = forward;

        //set power in the clockwise rotational direction
        powerLeft += rotate;
        powerRight += -rotate;
        //provide power to the motors
        driveLeft.setPower(clampMotor(powerLeft));
        driveRight.setPower(clampMotor(powerRight));

    }


    /**
     * Reset the encoder readings on all drive motors
     * @param enableEncoders if true, the motors will continue to have encoders active after reset
     */
    public void resetMotors(boolean enableEncoders){
        driveLeft.setPower(0);
        driveRight.setPower(0);
        driveLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (enableEncoders) {
            driveLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        else {
            driveLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            driveRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

    }


    public double clampMotor(double power) { return clampDouble(-1, 1, power); }


    /**
     * clamp a double value to put it within a given range
     * @param min lower bound of the range
     * @param max upper bound of the range
     * @param value double being clamped to the given range
     */
    public double clampDouble(double min, double max, double value)
    {
        double result = value;
        if(value > max)
            result = max;
        if(value < min)
            result = min;
        return result;
    }



    //////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////
    ////                                                                                  ////
    ////                           Variable Get/Set Method                                ////
    ////                                                                                  ////
    //////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////



    /**
     * retrieve the average value of ticks on all motors
     */
    public long getAverageTicks(){
        long averageTicks = (driveLeft.getCurrentPosition() + driveRight.getCurrentPosition())/4;
        return averageTicks;
    }


    /**
     * retrieve the average of the absolute value of ticks on all motors
     */
    public long getAverageAbsTicks(){
        long averageTicks = (Math.abs(driveLeft.getCurrentPosition()) + Math.abs(driveRight.getCurrentPosition()))/4;
        return averageTicks;
    }


    /**
     * clamp a double to match the power range of a motor
     * @param power the double value being clamped to motor power range
     */

    /**
     * assign the current heading of the robot to zero
     */
    public void setZeroHeading(){
        setHeading(0);
    }

    /**
     * assign the current heading of the robot to 45 (robot on field perimeter wall)
     */
    public void setWallHeading(){
        setHeading(45);
    }

    /**
     * assign the current heading of the robot to a specific angle
     * @param angle the value that the current heading will be assigned to
     */
    public void setHeading(double angle){
        poseHeading = angle;
        initialized = false; //triggers recalc of heading offset at next IMU update cycle
    }

    public void resetTPM(){
        forwardTPM = 2493;
    }

    /**
     * sets autonomous imu offset for turns
     */
    public void setAutonomousIMUOffset(double offset) {
        autonomousIMUOffset = offset;
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

    public double getBatteryVoltage(){
        return RC.h.voltageSensor.get("Motor Controller 1").getVoltage();
    }

    private long futureTime(float seconds){
        return System.nanoTime() + (long) (seconds * 1e9);
    }
}

