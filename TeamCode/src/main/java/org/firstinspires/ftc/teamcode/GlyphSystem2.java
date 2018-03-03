package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by tycho on 10/15/2017.
 */

public class GlyphSystem2 {

    DcMotor motorBelt = null;
    DcMotor motorLeft = null;
    DcMotor motorRight = null;
    Servo servoGripRight = null;
    Servo servoGripLeft = null;
    Servo servoBeltLeft = null;
    Servo servoBeltRight = null;
    Servo servoPhone = null;

    private int liftMax = 4000;
    private int liftStack = 2500; //stacking height
    private int liftMin = 50;
    private int liftAuto = 500;
    private int liftAuto2 = 1500;
    private int beltOn = 2000;
    private int beltOff = 1500;
    private int phoneUp = 900; //1250
    private int phoneDown = 2105; //1850
    private int liftPlanck = 450; //smallest distance to increment lift by when using runToPosition

    private double offsetHeading;
    private double offsetPitch;
    private double offsetRoll;

    public double pitch = 0;
    public double yaw = 0;
    public double roll = 0;

    BNO055IMU imu;
    Orientation imuAngles;


//    private int liftDown =


    boolean gripOpen = false;
    int gripOpenPos = 1350; //1544
    //int gripClosedPos = 1900;
    int gripClosedPos=1200; //1735
    int gripTightPos= 1100; //1700
    int gripWideOpenPos = 1450; //1381

    public GlyphSystem2(DcMotor motorLift, Servo servoGripRight, Servo servoGripLeft, DcMotor motorLeft, DcMotor motorRight, Servo servoBeltLeft, Servo servoBeltRight, Servo servoPhone, BNO055IMU imu){

        this.imu = imu;
        this.motorBelt = motorLift;
        this.servoGripRight = servoGripRight;
        this.servoGripLeft = servoGripLeft;
        this.motorLeft = motorLeft;
        this.motorRight = motorRight;
        this.servoBeltLeft = servoBeltLeft;
        this.servoBeltRight = servoBeltRight;
        this.servoPhone = servoPhone;
        this.servoBeltLeft.setPosition(servoNormalize(beltOff));
        this.servoBeltRight.setPosition(servoNormalize(beltOff));
        this.servoGripRight.setPosition(servoNormalize(gripClosedPos));

    }

    public void collect(){
        motorLeft.setPower(.90);
        motorRight.setPower(-.90);
    }

    public void hold(){

        motorLeft.setPower(0);
        motorRight.setPower(0);
    }

    public void toggleGrip(){
        if (gripOpen) {
            gripOpen = false;
            servoGripRight.setPosition(servoNormalize(gripClosedPos));
            servoGripLeft.setPosition(servoNormalize(gripClosedPos));
        }
        else {
            gripOpen = true;
            servoGripLeft.setPosition(servoNormalize(gripWideOpenPos));
            servoGripRight.setPosition(servoNormalize(gripWideOpenPos));
        }
    }
    public void closeGrip() {
        gripOpen = false;
        servoGripLeft.setPosition(servoNormalize(gripClosedPos));
        servoGripRight.setPosition(servoNormalize(gripClosedPos));
    }

    public void closeGripTight() {
        gripOpen = false;
        servoGripLeft.setPosition(servoNormalize(gripTightPos));
        servoGripRight.setPosition(servoNormalize(gripTightPos));
    }

    public void releaseGrip() {
        gripOpen = true;
        servoGripLeft.setPosition(servoNormalize(gripOpenPos));
        servoGripRight.setPosition(servoNormalize(gripOpenPos));
    }

    public void wideOpenGrip() {
        gripOpen = true;
        servoGripLeft.setPosition(servoNormalize(gripWideOpenPos));
        servoGripRight.setPosition(servoNormalize(gripWideOpenPos));
    }

    public void tiltPhoneUp(){
        servoPhone.setPosition(servoNormalize(phoneUp));
    }

    public void tiltPhoneDown(){
        servoPhone.setPosition(servoNormalize(phoneDown));
    }

    public void liftBelt () {
        servoBeltLeft.setPosition(servoNormalize(beltOn));
        servoBeltRight.setPosition(servoNormalize(beltOn));
    }


    public void stopInternalLift() {
        servoBeltLeft.setPosition(servoNormalize(beltOff));
        servoBeltRight.setPosition(servoNormalize(beltOff));
    }

    public void toggleBelt () {
        if(servoBeltRight.getPosition() == servoNormalize(beltOff)){
            servoBeltLeft.setPosition(servoNormalize(beltOn));
            servoBeltRight.setPosition(servoNormalize(beltOn));
            collect();

        }
        else{
            servoBeltLeft.setPosition(servoNormalize(beltOff));
            servoBeltRight.setPosition(servoNormalize(beltOff));
            hold();
        }
    }

    public void setMotorLeft (double pwr){
        motorLeft.setPower(pwr);
    }

    public void setMotorRight (double pwr){
        motorRight.setPower(pwr);
    }

    public void stopLift(){

        motorBelt.setPower(0);
        stopBelt();
        setMotorLeft(0);
        setMotorRight(0);

    }

    public void raiseLift(){
        if(motorBelt.getCurrentPosition() < liftMax) motorBelt.setPower(.5);
        else motorBelt.setPower(0);
    }
    public void lowerLift(){
        if(motorBelt.getCurrentPosition() > liftMin) motorBelt.setPower(-.5);
        else motorBelt.setPower(0);
    }

//    public void raiseLift2(){
//        if (motorBelt.getCurrentPosition() < liftMax && motorBelt.getTargetPosition() < liftMax) {
//            motorBelt.setTargetPosition((int) Math.min(motorBelt.getCurrentPosition() + liftPlanck, liftMax));
//            motorBelt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorBelt.setPower(1);
//        }
//    }
//    public void lowerLift2() {
//        if (motorBelt.getCurrentPosition() > liftMin && motorBelt.getTargetPosition() > liftMin) {
//            motorBelt.setTargetPosition((int) Math.max(motorBelt.getCurrentPosition() - liftPlanck, liftMin));
//            motorBelt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorBelt.setPower(.8);
//        }
//    }

    public void raiseLift2 (){
        motorBelt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBelt.setPower(.5);
    }

    public void lowerLift2 (){
        motorBelt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBelt.setPower(.5);
    }

    public void stopBelt() {
        motorBelt.setPower(0);
    }

    public void goLiftMax() {

            motorBelt.setTargetPosition(liftMax);
            motorBelt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBelt.setPower(1);

    }

    public void goLiftAuto() {

        motorBelt.setTargetPosition(liftAuto);
        motorBelt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBelt.setPower(1);

    }

    public void goLiftAuto2() {

        motorBelt.setTargetPosition(liftAuto2);
        motorBelt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBelt.setPower(1);

    }



    public void goLiftMin() {

        motorBelt.setTargetPosition(liftMin);
        motorBelt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBelt.setPower(1);

    }

    public void goLiftStack() {

        motorBelt.setTargetPosition(liftStack);
        motorBelt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBelt.setPower(1);

    }

    public int getMotorLiftPosition(){
        return motorBelt.getCurrentPosition();
    }

    public static double servoNormalize(int pulse){
        double normalized = (double)pulse;
        return (normalized - 750.0) / 1500.0; //convert mr servo controller pulse width to double on _0 - 1 scale
    }

    public void initIMU (){

        imuAngles= imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);

        offsetHeading = wrapAngleMinus(yaw, imuAngles.firstAngle);
        offsetRoll = wrapAngleMinus(imuAngles.secondAngle, roll);
        offsetPitch = wrapAngleMinus(imuAngles.thirdAngle, pitch);

    }

    public void update (){

        imuAngles= imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);

        yaw = wrapAngle(imuAngles.firstAngle, offsetHeading);
        pitch = wrapAngle(imuAngles.thirdAngle, offsetPitch);
        roll = wrapAngle(imuAngles.secondAngle, offsetRoll);
    }

    public double wrapAngle(double angle1, double angle2){
        return (angle1 + angle2) % 360;
    }
    public double wrapAngleMinus(double angle1, double angle2){
        return 360-((angle1 + angle2) % 360);
    }

}
