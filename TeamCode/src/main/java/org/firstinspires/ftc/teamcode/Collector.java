package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Math.abs;

/**
 * Created by 2938061 on 11/10/2017.
 */

public class Collector {

    //number of ticks per revolution REV HD motor: 2240
    //number of ticks per revolution REV Core    : 288

    DcMotor elbowLeft = null;
    DcMotor elbowRight = null;
    DcMotor extendABobLeft = null;
    DcMotor extendABobRight = null;

    Servo intakeRight = null;
    Servo intakeLeft = null;
    Servo hook = null;
    Servo intakeGate = null;

    int elbowPosInternal = 0;
    int elbowPos = 0;
    double elbowPwr = 0;

    int extendABobPosInternal = 0;
    int extendABobPos = 0;
    double extendABobPwr = 1;


    int servoHooked = 1560;
    int servoUnhooked = 2060;

    int servoGateOpen = 900;
    int servoGateClosed = 1495;

    public double intakePwr = .5;

    //normal Teleop encoder values
    public int pos_preIntake = 3600;
    public int pos_Intake   = 3900;
    public int pos_Deposit  = 1520;
    public int pos_reverseIntake = 80;
    public int pos_reversePreDeposit=1408;
    public int pos_reverseDeposit = 2908;
    public int pose_reverseSafeDrive = 1000;
    public int pos_PartialDeposit = 1700;
    public int pos_SafeDrive = 800;

    //autonomous encoder values
    public int pos_AutoPark = pos_SafeDrive + 500;
    public int pos_autonPrelatch = 2950;

    //end game encoder values
    public int pos_prelatch = 2558;
    public int pos_latched = 3023;
    public int pos_postlatch = 1240;

    //belt extension encoder values
    public static int extendMax = 2500;
    public static int extendMid= 980;
    public static int extendLow = 650; //clears hook and good for retracting prior to deposit without tipping robot
    public static int extendMin = 300;  //prevent crunching collector tray
    public static int extendPreLatch = extendMax;

    //trueArticulation tolerances
    public static int extendABobTolerance = 15;
    public static int elbowTolerance = 15;

    //filler value; needs to be updated to reflect actual ratio
    public double ticksPerDegree = 5;

    public boolean active = true;

    public Collector(DcMotor elbowLeft, DcMotor elbowRight, DcMotor extendABobLeft, DcMotor extendABobRight, Servo intakeRight, Servo intakeLeft, Servo hook, Servo intakeGate){

        elbowLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbowRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbowRight.setDirection(DcMotorSimple.Direction.REVERSE);

        extendABobLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendABobRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendABobRight.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeLeft.setDirection(Servo.Direction.REVERSE);

        this.elbowLeft = elbowLeft;
        this.elbowRight = elbowRight;
        this.extendABobLeft = extendABobLeft;
        this.extendABobRight = extendABobRight;
        this.intakeRight = intakeRight;
        this.intakeLeft = intakeLeft;
        this.hook = hook;
        this.intakeGate = intakeGate;

    }

    public void update(){
        if(active && elbowPosInternal!=elbowPos) { //don't keep updating if we are retractBelt to target position
            elbowPosInternal = elbowPos;
            elbowLeft.setTargetPosition(elbowPos);
            elbowRight.setTargetPosition(elbowPos);
            elbowLeft.setPower(elbowPwr);
            elbowRight.setPower(elbowPwr);
        }
        if(active && extendABobPosInternal!=extendABobPos) { //don't keep updating if we are retractBelt to target position
            extendABobPosInternal = extendABobPos;
            extendABobLeft.setTargetPosition(extendABobPos);
            extendABobRight.setTargetPosition(extendABobPos);
            extendABobLeft.setPower(extendABobPwr);
            extendABobRight.setPower(extendABobPwr);
        }
    }

    public void hookOn(){
        hook.setPosition(servoNormalize(servoHooked));
    }
    public void hookOff(){
        hook.setPosition(servoNormalize(servoUnhooked));
    }

    public void openGate(){
        intakeGate.setPosition(servoNormalize(servoGateOpen));
    }
    public void closeGate(){
        intakeGate.setPosition(servoNormalize(servoGateClosed));
    }


    public void collect(){ intakeRight.setPosition(.5 + intakePwr);
        intakeLeft.setPosition(.5 + intakePwr);
        intakeRight.setPosition(.5 + intakePwr);
    }
    public void eject(){
        intakeRight.setPosition(.5 - intakePwr);
        intakeLeft.setPosition(.5 - intakePwr);}
    public void stopIntake(){
        intakeRight.setPosition(.5);
        intakeLeft.setPosition(.5);
    }

    public boolean isActive(){
        return active;
    }

    public void setExtendABobTargetPos(int pos){
        extendABobPos = pos;
    }
    public int getExtendABobTargetPos(){
        return extendABobPos;
    }
    public int getExtendABobCurrentPos(){
        return extendABobLeft.getCurrentPosition();
    }
    public void setExtendABobPwr(double pwr){ extendABobPwr = pwr; }

    public void setElbowTargetPos(int pos){
        elbowPos = pos;
    }
    public boolean setElbowTargetPos(int pos, double speed){
        setElbowTargetPos(pos);
        setElbowPwr(speed);
        if (nearTargetElbow()) return true;
        else return false;
    }
    public int getElbowTargetPos(){
        return elbowPos;
    }
    public int getElbowCurrentPos(){
        return elbowLeft.getCurrentPosition();
    }
    public int getElbowCurrentPos2() {
        return elbowRight.getCurrentPosition();
    }
    public int getElbowCurrentPosAvg() {
        return (getElbowCurrentPos() + getElbowCurrentPos2())/2;
    }
    public void setElbowPwr(double pwr){ elbowPwr = pwr; }

    public void kill(){
        setElbowPwr(0);
        setExtendABobPwr(0);
        update();
        active = false;
    }
    public void restart(double elbowPwr, double extendABobPwr){
        setElbowPwr(elbowPwr);
        setExtendABobPwr(extendABobPwr);
        active = true;
    }

    public void resetEncoders() {
        //just encoders - only safe to call if we know collector is in normal starting position
        elbowLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendABobLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendABobRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbowRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendABobRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendABobLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public boolean extendToMin(){
        return extendToMin(extendABobPwr, 15);
    }
    public boolean extendToMin(double speed, int range){
        setExtendABobPwr(speed);
        setExtendABobTargetPos(extendMin);
        if((Math.abs(getExtendABobCurrentPos()-getExtendABobTargetPos()))<range){
            return true;
        }
        return false;
    }
    public boolean extendToLow(){
        return extendToLow(extendABobPwr, 15);
    }
    public boolean extendToLow(double speed, int range){
        setExtendABobPwr(speed);
        setExtendABobTargetPos(extendLow);
        if((Math.abs(getExtendABobCurrentPos()-getExtendABobTargetPos()))<range){
            return true;
        }
        return false;
    }
    public boolean extendToMid(){
        return extendToMid(extendABobPwr, 15);
    }
    public boolean extendToMid(double speed, int range){
        setExtendABobPwr(speed);
        setExtendABobTargetPos(extendMid);
        if((Math.abs(getExtendABobCurrentPos()-getExtendABobTargetPos()))<range){
            return true;
        }
        return false;
    }
    public boolean extendToMax(){
        return extendToMax(extendABobPwr, 15);
    }
    public boolean extendToMax(double speed, int range){
        setExtendABobPwr(speed);
        setExtendABobTargetPos(extendMax);
        if((Math.abs(getExtendABobCurrentPos()-getExtendABobTargetPos()))<range){
            return true;
        }
        return false;
    }

    public boolean nearTargetExtend(){
        if((Math.abs(getExtendABobCurrentPos()-getExtendABobTargetPos()))<extendABobTolerance) return true;
        else return false;
    }
    public boolean nearTargetElbow(){
        if ((Math.abs( getElbowCurrentPos()-getElbowTargetPos()))<elbowTolerance) return true;
        else return false;
    }
    public boolean nearTarget(){
        if (nearTargetElbow() && nearTargetExtend()) return true;
        else return false;
    }

    public void increaseElbowAngle(){
        setElbowTargetPos(Math.min(getElbowCurrentPos() + 100, pos_Intake));
    }
    public void decreaseElbowAngle(){
        setExtendABobTargetPos(Math.max(getExtendABobCurrentPos() - 100, extendMin));
    }

    public void extendBelt(){
        setExtendABobTargetPos(Math.min(getExtendABobCurrentPos() + 100, extendMax));
    }
    public void retractBelt(){
        setElbowTargetPos(Math.max(getElbowCurrentPos() - 100, 0));
    }

    public void runToAngle(double angle){
        setElbowTargetPos((int)(angle * ticksPerDegree));
    }//untested

    public static double servoNormalize(int pulse){
        double normalized = (double)pulse;
        return (normalized - 750.0) / 1500.0; //convert mr servo controller pulse width to double on _0 - 1 scale
    }
}
