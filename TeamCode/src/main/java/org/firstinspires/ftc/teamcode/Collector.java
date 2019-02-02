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

    int elbowPosInternal = 0;
    int elbowPos = 0;
    double elbowPwr = 0;

    int extendABobPosInternal = 0;
    int extendABobPos = 0;
    double extendABobPwr = .90;



    //all filler values; need to be updated to reflect actual positions

    public int posIntake   = 4200;
    public int posDeposit  = 3231;
    public int posPreLatch = 2025;
    public int posLatch    = 2718;
    public int posPostLatch = 20;
    public int posSafeDrive = 794;
    public double intakePwr = .5;

    public static int extendMax = 3700;
    public static int extendMid= 1750;
    public static int extendMin = 10;


    //filler value; needs to be updated to reflect actual ratio
    public double ticksPerDegree = 5;

    public boolean active = true;

    public Collector(DcMotor elbowLeft, DcMotor elbowRight, DcMotor extendABobLeft, DcMotor extendABobRight, Servo intakeRight, Servo intakeLeft){

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


    public boolean beltMin(){
        setExtendABobTargetPos(extendMin);
        if(getExtendABobCurrentPos() == extendMin)
            return true;
        return false;
    }
    public boolean beltMid(){
        setExtendABobTargetPos(extendMid);
        if(getExtendABobCurrentPos() == extendMid)
            return true;
        return false;
    }
    public boolean beltMax(){
        setExtendABobTargetPos(extendMax);
        if(getExtendABobCurrentPos() == extendMax)
            return true;
        return false;
    }





    public void open(){
        setElbowTargetPos(Math.min(getElbowCurrentPos() + 100, posIntake));
    }

    public void retract(){
        setExtendABobTargetPos(Math.max(getExtendABobCurrentPos() - 100, extendMin));
    }

    public void extend(){
        setExtendABobTargetPos(Math.min(getExtendABobCurrentPos() + 100, extendMax));
    }

    public void close(){
        setElbowTargetPos(Math.max(getElbowCurrentPos() - 100, posPostLatch));
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
