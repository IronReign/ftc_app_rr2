package org.firstinspires.ftc.teamcode;

//import org.swerverobotics.library.interfaces.*;

//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;

/**
 * The Pose class stores the current real world position/orientation: <b>position</b>, <b>heading</b>,
 * and <b>speed</b> of the robot.
 *
 * This class should be a point of reference for any navigation classes that want to know current
 * orientation and location of the robot.  The update method must be called regularly, it
 * monitors and integrates data from the orientation (IMU) and odometry (motor encoder) sensors.
 * @author Max Virani, Eliot Partridge,
 * @version _0.2
 * @since 2015-10-17
 */

public class Pose
{


//    private double poseX;
//    private double poseY;
//    private double poseHeading;
//    private double poseSpeed;
//    private double poseFacing;
//    private double posePitch;
//    private double poseRoll;
//    private long timeStamp; //timestamp of last update
//    private boolean initialized = false;
//    private double offsetHeading;
//    private double offsetPitch;
//    private double offsetRoll;
//    private long ticksPerMeterFL = 20000; //Front left. arbitrary initial value so we don't get a divide by zero
//    private long ticksPerMeterFR = 20000; //Front right. need actual measured value
//    private long ticksPerMeterBL = 20000; //Back left. Need measured value
//    private long ticksPerMeterBR = 20000; //Back right. Need measured value
//    private long ticksPerSpin = 20000; //Average ticks/motor to spin entire robot in a circle. May be changed to degrees. Need measured value
//    private long ticksPerMeterClimber = 11687; //actual measured value
//    private long ticksPerInchClimber = (long)(ticksPerMeterClimber / 39.3701); //used to verify the tick values vs the tape measure
//    private long ticksClimberOffset = 0;
//    private long ticksLeftPrev;
//    private long ticksRightPrev;
//    private long ticksLeftOffset; //provide a way to offset (effectively reset) the motor encoder readings
//    private long ticksRightOffset;
//    private double wheelbase; //the width between the wheels (center of tire to center of tire)
//    private double wheeldist; //from front wheel to back wheel (axle to axle)
//    public DcMotor flinger;
//    protected boolean flingerRelaxing = false;
//    protected boolean flingerFlinging = false;
//    final protected double flingerRelaxPwr = 0.075;
//    final protected double flingerFlingPwr = -1;
//    long flingerTimer;
//    protected boolean conveyorActive = false;
//    protected boolean conveyorReverse = false;
//    final protected double conveyorPwr = 0.5 //Test to find best value
///*
//
//
//    /**
//     * Create a Pose instance that stores all real world position/orientation elements:
//     * <var>x</var>, <var>y</var>, <var>heading</var>, and <var>speed</var>.
//     *
//     * @param x     The position relative to the x axis of the field
//     * @param y     The position relative to the y axis of the field
//     * @param heading The heading of the robot
//     * @param speed The speed of the robot
//     */
//    public Pose(double x, double y, double heading, double speed, DcMotor fling)
//    {
//
//        poseX     = x;
//        poseY     = y;
//        poseHeading = heading;
//        poseSpeed = speed;
//        posePitch = 0;
//        poseRoll = 0;
//        flinger = fling;
//    }
//
//    /**
//     * Creates a Pose instance with _0 speed, to prevent muscle fatigue
//     * by excess typing demand on the software team members. This is likely
//     * the one to use on cliffInit when speed is zero and starting position is known
//     *
//     * @param x     The position relative to the x axis of the field
//     * @param y     The position relative to the y axis of the field
//     * @param angle The angle of the robot
//     */
//    public Pose(double x, double y, double angle, DcMotor fling)
//    {
//
//        poseX     = x;
//        poseY     = y;
//        poseHeading = angle;
//        poseSpeed = 0;
//        flinger = fling;
//    }
//
//    /**
//     * Creates a base Pose instance at the origin, (_0,_0), with _0 speed and _0 angle.
//     * Useful for determining the Pose of the robot relative to the origin.
//     */
//    public Pose(DcMotor fling)
//    {
//
//        poseX     = 0;
//        poseY     = 0;
//        poseHeading = 0;
//        poseSpeed = 0;
//        posePitch=0;
//        poseRoll=0;
//        flinger = flingLeft;
//    }
//
//
//    /**
//     * Set the current position of the robot in the X direction on the field
//      * @param poseX
//     */
//    public void setPoseX(double poseX) {
//        this.poseX = poseX;
//    }
//
//    /**
//     * Set the current position of the robot in the Y direction on the field
//     * @param poseY
//     */
//    public void setPoseY(double poseY) {
//        this.poseY = poseY;
//    }
//
//    /**
//     * Set the absolute heading (yaw) of the robot _0-360 degrees
//     * @param poseHeading
//     */
//    public void setPoseHeading(double poseHeading) {
//        this.poseHeading = poseHeading;
//        initialized = false; //trigger recalc of offset on next Update
//    }
//
//    /**
//     * Set the absolute pitch of the robot _0-360 degrees
//     * @param posePitch
//     */
//    public void setPosePitch(double posePitch) {
//        this.posePitch = posePitch;
//        initialized = false; //trigger recalc of offset on next Update
//    }
//
//    /**
//     * Set the absolute roll of the robot _0-360 degrees
//     * @param poseRoll
//     */
//    public void setPoseRoll(double poseRoll) {
//        this.poseRoll = poseRoll;
//        initialized = false; //trigger recalc of offset on next Update
//    }
//    /**
//     * Returns the x position of the robot
//     *
//     * @return The current x position of the robot
//     */
//    public double getX()
//    {
//        return poseX;
//    }
//
//    /**
//     * Returns the y position of the robot
//     *
//     * @return The current y position of the robot
//     */
//    public double getY()
//    {
//        return poseY;
//    }
//
//    /**
//     * Returns the angle of the robot
//     *
//     * @return The current angle of the robot
//     */
//    public double getHeading()
//    {
//        return poseHeading;
//    }
//
//    /**
//     * Returns the speed of the robot
//     *
//     * @return The current speed of the robot
//     */
//    public double getSpeed()
//    {
//        return poseSpeed;
//    }
//    public double getPitch() {
//        return posePitch;
//    }
//
//    public double getRoll() {
//        return poseRoll;
//    }
//
//
//
//    public void setTicksPerMeterFL(long ticksPerMeterFL) { this.ticksPerMeterFL = ticksPerMeterFL; }
//    public void setTicksPerMeterBL(long ticksPerMeterBL) { this.ticksPerMeterBL = ticksPerMeterBL; }
//    public void setTicksPerMeterFR(long ticksPerMeterFR) { this.ticksPerMeterFR = ticksPerMeterFR; }
//    public void setTicksPerMeterBR(long ticksPerMeterBR) { this.ticksPerMeterBR = ticksPerMeterBR; }
//
//    public long getTicksPerMeterFL() {
//        return ticksPerMeterFL;
//    }
//    public long getTicksPerMeterBL() { return ticksPerMeterBL; }
//    public long getTicksPerMeterFR() {
//        return ticksPerMeterFR;
//    }
//    public long getTicksPerMeterBR() {
//        return ticksPerMeterBR;
//    }
//
//
//    public boolean fling(){ //returns true when complete - any subsequent call will restart the fling
//        return flingerMove(.15, flingerFlingPwr);
//    }
//
//    public boolean flingRelax(){//returns true when complete - any subsequent call will restart the relax
//        return flingerMove(1, flingerRelaxPwr);
//    }
//
//
//    private boolean flingerMove(double seconds, double power) //returns true when move is complete - so don't call after it returns true unless we want to start a fresh move
//    {
//        if (((long)flingerTimer)==0){ //this needs to be true only the first time in
//            flingerTimer=System.nanoTime() + (long)(seconds * 1e9); //set expiration time
//            flinger.setPower(power);
//
//
//        }
//        if (flingerTimer < System.nanoTime()) //timer has expired
//        {
//            flinger.setPower(0);
//
//            flingerTimer=0;
//            return true;
//        }
//
//        return false; //do nothing - motors should already be moving at specified power
//
//    }
//
//
//    protected int flingCount=0;
//
//
//    public boolean flingerWiggle(){ //return true when settled
//        if (System.nanoTime() > flingerTimer ) {
//            if (flingCount < 20) {
//                if ((flingCount & 1) == 0) {//upstroke on even
//                    flinger.setPower(-.1);
//
//                } else { //downstroke on odds
//                    flinger.setPower(.2);
//
//                }
//                flingerTimer = System.nanoTime() + (long).5e8;
//                flingCount++;
//                return false;
//            }
//            else {
//                flingCount = 0; //reset in case we need to re-wiggle later
//                flingerTimer=0;
//                return true;
//            }
//        }
//        else return false;
//
//    }
//
//    /**
//     * Update the current location of the robot. This implementation gets heading and orientation
//     * from the Bosch BNO055 IMU and assumes a simple differential steer robot with left and right motor
//     * encoders.
//     *
//     *
//     * The current naive implementation assumes an unobstructed robot - it cannot account
//     * for running into objects and assumes no slippage in the wheel encoders.  Debris
//     * on the field and the mountain ramps will cause problems for this implementation. Further
//     * work could be done to compare odometry against IMU integrated displacement calculations to
//     * detect stalls and slips
//     *
//     * This method should be called regularly - about every 20 - 30 milliseconds or so.
//     *
//     * @param imu
//     * @param ticksLeft
//     * @param ticksRight
//     */
//    public void Update(EulerAngles imu, long ticksLeft, long ticksRight){
//        long currentTime = System.nanoTime();
//        if (!initialized){
//            //first time in - we assume that the robot has not started moving and that orientation values are set to the current absolute orientation
//            //so first set of imu readings are effectively offsets
//            offsetHeading = wrapAngleMinus(poseHeading, imu.heading);
//            offsetPitch = wrapAngleMinus(imu.pitch, posePitch);
//            offsetRoll = wrapAngleMinus(imu.roll, poseRoll);
//            initialized = true;
//        }
//
//        //poseHeading = imu.heading + offsetHeading;
//        //posePitch = imu.pitch + offsetPitch;
//        //poseRoll = imu.roll + offsetRoll;
//
//
//        poseHeading = wrapAngle(imu.heading, offsetHeading);
//        posePitch = wrapAngle(imu.pitch, offsetPitch);
//        poseRoll = wrapAngle(imu.roll, offsetRoll);
//
//        double displacementX;
//        double displacementY
//        //double displacement = (((double)(ticksRight - ticksRightPrev)/ticksPerMeterRight) + ((double)(ticksLeft - ticksLeftPrev)/ticksPerMeterLeft))/2.0;
//
//        poseSpeed = displacement / (double)(currentTime - this.timeStamp)*1000000; //meters per second when ticks per meter is calibrated
//
//        timeStamp = currentTime;
//        ticksRightPrev = ticksRight;
//        ticksLeftPrev = ticksLeft;
//
//        double poseHeadingRad = Math.toRadians(poseHeading);
//
//        poseX += displacement * Math.cos(poseHeadingRad);
//        poseY += displacement * Math.sin(poseHeadingRad);
//
//        /*
//        if (flingerFlinging){
//            flingerTimer = currentTime + (long).15e9; //.15 seconds into the future
//            flingerFlinging = false;
//        }
//
//        if (flingerRelaxing){
//            flingerTimer = currentTime + (long)1e9; //1 seconds into the future
//            flingerRelaxing = false;
//        }
//
//        if (currentTime> flingerTimer){
//            flingerTimer = 0;
//            flingerLeft.setPower(0);
//            flingerRight.setPower(0);
//
//
//        }
//        */
//    }
//
//    public long getTicksLeftPrev()
//    {
//        return ticksLeftPrev;
//    }
//    public long getTicksRightPrev()
//    {
//        return ticksRightPrev;
//    }
//
//    /**
//     *
//     * @returns odometer value
//     */
//    public double getOdometer() {
//
//    return  (((double)(ticksRightPrev - ticksRightOffset)/ticksPerMeterRight) + ((double)(ticksLeftPrev - ticksLeftOffset)/ticksPerMeterLeft))/2.0;
//
//    }
//
//    /**
//     *
//      * @param distance
//     */
//    public void setOdometer(double distance){
//        ticksRightOffset = ticksRightPrev + (long) (distance * ticksPerMeterRight);
//        ticksLeftOffset = ticksLeftPrev + (long) (distance * ticksPerMeterLeft);
//    }
//
//    /**
//     * returns the minimum difference (in absolute terms) between two angles,
//     * preserves the sign of the difference
//     *
//     * @param angle1
//     * @param angle2
//     * @return
//     */
//    public double diffAngle(double angle1, double angle2){
//        return Math.abs(angle1 - angle2) < Math.abs(angle2-angle1) ? Math.abs(angle1 - angle2) : Math.abs(angle2-angle1);
//    }
//
//    public double diffAngle2(double angle1, double angle2){
//
//        double diff = angle1 - angle2;
//
//        //allow wrap around
//
//            if (Math.abs(diff) > 180)
//                    {
//                if (diff > 0) {
//                    diff -= 360;
//                } else {
//                    diff += 360;
//                }
//            }
//        return diff;
//        }
//
//
//    /**
//     * Apply and angular adjustment to a base angle with result wrapping around at 360 degress
//     *
//      * @param angle1
//     * @param angle2
//     * @return
//     */
//    public double wrapAngle(double angle1, double angle2){
//        return (angle1 + angle2) % 360;
//    }
//    public double wrapAngleMinus(double angle1, double angle2){
//        return 360-((angle1 + angle2) % 360);
//    }
//
//    double getBearingTo(double x, double y){
//        double diffx = x-poseX;
//        double diffy = y-poseY;
//        return ( Math.toDegrees(Math.atan2( diffy, diffx)) - 90  + 360 ) % 360;
//    }
//
//    double getBearingOpposite(double x, double y){
//        double diffx = x-poseX;
//        double diffy = y-poseY;
//        return ( Math.toDegrees(Math.atan2( diffy, diffx)) + 90 + 360 ) % 360;
//    }
//
//    double getDistanceTo(double x, double y){
//
//            double dx = x - poseX;
//            double dy = y - poseY;
//            return Math.sqrt(dx*dx + dy*dy);
//
//        }
//
//
//    public double ServoNormalize(int pulse){
//        double normalized = (double)pulse;
//        return (normalized - 750.0) / 1500.0; //convert mr servo controller pulse width to double on _0 - 1 scale
//    }


}
