package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.Pose.ticksPerRot;

/**
 * Created by Tycho on 1/7/2017.
 */

public class CapTrap {
    DcMotor motorLift = null;

    static double spoolDiameter       = .05; //diameter of the spool in meters
    private double spoolCircumference = spoolDiameter *Math.PI;
    private int ticksPerM             = (int)(ticksPerRot*(1/spoolCircumference));

    private double hSummit = 0.88;
    private double hDeposit = 0.61;
    private double hStarting = 0;
    private double hPickup = -0.34;

    public CapTrap(DcMotor motorLift){
        this.motorLift = motorLift;
        this.motorLift.setMaxSpeed(1000);
    }
    public void resetMotor(boolean runToPosition){
        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if(runToPosition) motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        else motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorLift.setMaxSpeed(1000);
    }
    public void summit(){
        setRunmode(DcMotor.RunMode.RUN_TO_POSITION, 1000, 1);
        motorLift.setPower(1);
        setPositionMeters(hSummit); //TODO: set number of meters to reflect actual height needed to score the cap ball
    }
    public void deposit(){
        setRunmode(DcMotor.RunMode.RUN_TO_POSITION, 1000, 1);
        motorLift.setPower(1);
        setPositionMeters(hDeposit); //TODO: set number of meters to reflect actual height needed to score the cap ball
    }
    public void setZero(DcMotor.RunMode runMode,int tps, double pwr){
        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setRunmode(runMode, tps, pwr);
    }
    public void startingPos(){
        setRunmode(DcMotor.RunMode.RUN_TO_POSITION, 1000, 1);
        motorLift.setPower(1);
        setPositionMeters(hStarting);
    }
    public void pickupPos(){
        setRunmode(DcMotor.RunMode.RUN_TO_POSITION, 1000, 1);
        motorLift.setPower(1);
        setPositionMeters(hPickup);
    }
    public double getPositionMeters(){
        return motorLift.getTargetPosition()/ticksPerM;
    }

    public void cycleUp(){
        if(hPickup - 0.1 < getPositionMeters() && getPositionMeters() < hPickup + 0.1){
            startingPos();
        }
        else if(hStarting - 0.1 < getPositionMeters() && getPositionMeters() < hStarting + 0.1){
            deposit();
        }
        else if(hDeposit - 0.1 < getPositionMeters() && getPositionMeters() < hDeposit + 0.1){
            summit();
        }
        else if(hSummit - 0.1 < getPositionMeters() && getPositionMeters() < hSummit + 0.1){

        }
        else{
            startingPos();
        }
    }

    public void cycleDown(){
        if(hPickup - 0.1 < getPositionMeters() && getPositionMeters() < hPickup + 0.1){

        }
        else if(hStarting - 0.1 < getPositionMeters() && getPositionMeters() < hStarting + 0.1){
            pickupPos();
        }
        else if(hDeposit - 0.1 < getPositionMeters() && getPositionMeters() < hDeposit + 0.1){
            startingPos();
        }
        else if(hSummit - 0.1 < getPositionMeters() && getPositionMeters() < hSummit + 0.1){
            deposit();
        }
        else{
            startingPos();
        }
    }

    public void setPositionMeters(double m){
        if(!(motorLift.getMode().equals(DcMotor.RunMode.RUN_TO_POSITION))) motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m *= ticksPerM;
        motorLift.setTargetPosition((int)m);
    }
    public void setRunmode(DcMotor.RunMode runMode, int tps, double pwr){
        motorLift.setMode(runMode);
        motorLift.setMaxSpeed(tps);
        motorLift.setPower(pwr);
    }
    public void raise(double pwr){
        setRunmode(DcMotor.RunMode.RUN_USING_ENCODER, 10000, 1);
    }
    public void lower(double pwr){
        setRunmode(DcMotor.RunMode.RUN_USING_ENCODER, 10000, -1);
    }
    public void stop(){
        motorLift.setPower(0);
    }
}
