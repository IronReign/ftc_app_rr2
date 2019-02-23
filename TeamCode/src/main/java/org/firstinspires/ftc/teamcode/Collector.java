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

    public int pos_preIntake = 3600;
    public int pos_Intake   = 3900;
    public int pos_Deposit  = 1520;
    public int pos_PartialDeposit = 1700;
    public int pos_SafeDrive = 800;
    public int pos_AutoPark = pos_SafeDrive + 300;
    public int pos_autonPrelatch = 2950;
    public int pos_prelatch = 2558; //endgame preLatch
    public int pos_latched = 3023; //todo - likely needs to be same as prelatch
    public int pos_postlatch = 1240; //todo - check for safety - but might work
    public int pos_Deployed = 0; //todo - what is this value?

    public static int extendMax = 2500;
    public static int extendMid= 980;
    public static int extendLow = 650; //clears hook and good for retracting prior to deposit without tipping robot
    public static int extendMin = 300;  //prevent crunching collector tray
    public static int extendPreLatch = extendMax;

    //filler value; needs to be updated to reflect actual ratio
    public double ticksPerDegree = 5;

    public boolean active = true;

    public Collector(DcMotor elbowLeft, DcMotor elbowRight, DcMotor extendABobLeft, DcMotor extendABobRight, Servo intakeRight, Servo intakeLeft, Servo hook, Servo intakeGate){

        elbowLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbowRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //elbowLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        elbowRight.setDirection(DcMotorSimple.Direction.REVERSE);

        extendABobLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendABobRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendABobRight.setDirection(DcMotorSimple.Direction.REVERSE);


        this.elbowLeft = elbowLeft;
        this.elbowRight = elbowRight;
        this.extendABobLeft = extendABobLeft;
        this.extendABobRight = extendABobRight;
        this.intakeRight = intakeRight;
        this.intakeLeft = intakeLeft;
        this.hook = hook;
        this.intakeGate = intakeGate;
        intakeLeft.setDirection(Servo.Direction.REVERSE);

    }

    public void update(){
        if(active && elbowPosInternal!=elbowPos) { //don't keep updating if we are close to target position
            elbowPosInternal = elbowPos;
            elbowLeft.setTargetPosition(elbowPos);
            elbowRight.setTargetPosition(elbowPos);
            elbowLeft.setPower(elbowPwr);
            elbowRight.setPower(elbowPwr);
        }
        if(active && extendABobPosInternal!=extendABobPos) { //don't keep updating if we are close to target position
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
        //return elbowRight.getCurrentPosition();
    }
    public int getElbowCurrentPos2(){
        return elbowRight.getCurrentPosition();
        //return elbowRight.getCurrentPosition();
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
        if((Math.abs(getExtendABobCurrentPos()-getExtendABobTargetPos()))<15) return true;
        else return false;
    }
    public boolean nearTargetElbow(){
        if ((Math.abs( getElbowCurrentPos()-getElbowTargetPos()))<15) return true;
        else return false;
    }
    public boolean nearTarget(){
        if (nearTargetElbow() && nearTargetExtend()) return true;
        else return false;
    }

    public void open(){
        setElbowTargetPos(Math.min(getElbowCurrentPos() + 100, pos_Intake));
    }

    public void retract(){
        setExtendABobTargetPos(Math.max(getExtendABobCurrentPos() - 100, extendMin));
    }

    public void extend(){
        setExtendABobTargetPos(Math.min(getExtendABobCurrentPos() + 100, extendMax));
    }

    public void close(){
        setElbowTargetPos(Math.max(getElbowCurrentPos() - 100, 0));
    }


    public void runToAngle(double angle){
        setElbowTargetPos((int)(angle * ticksPerDegree));
    }

    public static double servoNormalize(int pulse){
        double normalized = (double)pulse;
        return (normalized - 750.0) / 1500.0; //convert mr servo controller pulse width to double on _0 - 1 scale
    }

    long futureTime(float seconds){
        return System.nanoTime() + (long) (seconds * 1e9);
    }

}
