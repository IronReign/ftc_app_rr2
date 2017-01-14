package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Tycho on 1/7/2017.
 */

public class CapTrap {
    DcMotor motorLift = null;
    static int ticksPerM = 1000; //TODO: set number of ticks to reflect actual ticks per meter on the lift
    public CapTrap(DcMotor motorLift){
        this.motorLift = motorLift;
        this.motorLift.setMaxSpeed(10000);
    }
    public void resetMotor(boolean runToPosition){
        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if(runToPosition) motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        else motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorLift.setMaxSpeed(10000);
    }
    public void liftToScoringPosition(){
        setRunmode(DcMotor.RunMode.RUN_TO_POSITION, 10000, 1);
        motorLift.setPower(1);
        setPositionMeters(1.5); //TODO: set number of meters to reflect actual height needed to score the cap ball
    }
    public void lowerToStartingPos(){
        setRunmode(DcMotor.RunMode.RUN_TO_POSITION, 10000, 1);
        motorLift.setPower(1);
        setPositionMeters(0);
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
