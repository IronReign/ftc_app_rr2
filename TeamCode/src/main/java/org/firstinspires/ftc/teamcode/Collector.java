package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static java.lang.Math.abs;

/**
 * Created by 2938061 on 11/10/2017.
 */

public class Collector {

    //number of ticks per revolution REV HD motor: 2240
    //number of ticks per revolution REV Core    : 288

    DcMotor elbowLeft = null;
    DcMotor elbowRight = null;
    DcMotor intake = null;

    int elbowPosInternal = 0;
    int elbowPos = 0;
    double elbowPwr = 0;


    //all filler values; need to be updated to reflect actual positions
    public int posIntake   = 4200;
    public int posDeposit  = 100;
    public int posPreLatch = 100;
    public int posLatch    = 0;
    public int posSuperman = 100;

    //filler value; needs to be updated to reflect actual ratio
    public double ticksPerDegree = 5;

    public boolean active;

    public Collector(DcMotor elbowLeft, DcMotor elbowRight, DcMotor intake){

        elbowLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbowRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbowLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        elbowRight.setDirection(DcMotorSimple.Direction.FORWARD);

        this.elbowLeft = elbowLeft;
        this.elbowRight = elbowRight;
        this.intake = intake;

    }

    public void update(){
        if(active && elbowPosInternal!=elbowPos) { //don't keep updating if we are close to target position
            elbowPosInternal = elbowPos;
            elbowLeft.setTargetPosition(elbowPos);
            elbowRight.setTargetPosition(elbowPos);
            elbowLeft.setPower(elbowPwr);
            elbowRight.setPower(elbowPwr);
        }
    }

    public boolean isActive(){
        return active;
    }

    public void setTargetPosition(int pos){
        elbowPos = pos;
    }

    public int getTargetPosition(){
        return elbowPos;
    }

    public int getCurrentPosition(){
        return elbowLeft.getCurrentPosition();
    }

    public void setPower(double pwr){
        elbowPwr = pwr;
    }

    public void kill(){
        setPower(0);
        update();
        active = false;
    }

    public void restart(double pwr){
        setPower(pwr);
        active = true;
    }

    public void advance(){
        setTargetPosition(getCurrentPosition() + 60);
    }

    public void retreat(){
        setTargetPosition(getCurrentPosition() - 60);
    }

    public void runToAngle(double angle){
        setTargetPosition((int)(angle * ticksPerDegree));
    }

    public static double servoNormalize(int pulse){
        double normalized = (double)pulse;
        return (normalized - 750.0) / 1500.0; //convert mr servo controller pulse width to double on _0 - 1 scale
    }

    long futureTime(float seconds){
        return System.nanoTime() + (long) (seconds * 1e9);
    }

}
