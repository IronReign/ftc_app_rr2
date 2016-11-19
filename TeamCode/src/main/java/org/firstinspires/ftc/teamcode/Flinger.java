package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by tycho on 11/18/2016.
 */

public class Flinger {
    private int position = 0; //range of 1-1120
    private int speed = 0; //ticks per second
    private int backupSpeed = -1;
    DcMotor motorFlinger = null;
    private double power = 0;
    static int ticksPerRot = 1120;

    public Flinger(){
        position = 0;
    }
    public Flinger(DcMotor motorFlinger){
        position = 0;
        this.motorFlinger = motorFlinger;
        resetFlinger();
    }
    public Flinger(int ticksPerSec, DcMotor motorFlinger){
        position = 0;
        this.speed = ticksPerSec;
        this.motorFlinger = motorFlinger;
        this.motorFlinger.setPower(1);
        resetFlinger();
    }
    public Flinger(int ticksPerSec,int position, DcMotor motorFlinger){
        this.speed = ticksPerSec;
        this.position = position;
        this.motorFlinger = motorFlinger;
        this.motorFlinger.setPower(1);
        resetFlinger();
    }
    public void setPosition(int position){
        this.position = position;
        runToPosition();
    }
    public void setSpeed(int ticksPerSec){
        this.speed = ticksPerSec;
        this.motorFlinger.setMaxSpeed(speed);
    }
    public void runToPosition(){
        this.motorFlinger.setTargetPosition(position);
    }
    public void pullBack(){
        resetFlinger();
        setPosition(ticksPerRot/2);
    }
    public void resetFlinger(){
        motorFlinger.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFlinger.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFlinger.setMaxSpeed(speed);
        setPosition(0);
    }
    public void fling(){
        int pos = position + ticksPerRot;
        setPosition(pos);
    }
    public void emergencyStop(){
        backupSpeed = speed;
        motorFlinger.setPower(0);
        setSpeed(0);
    }
    public boolean isStopped(){
        return backupSpeed >= 0;
    }
    public void restart(){
        setSpeed(backupSpeed);
        motorFlinger.setPower(1);
        backupSpeed = -1;
    }
}
