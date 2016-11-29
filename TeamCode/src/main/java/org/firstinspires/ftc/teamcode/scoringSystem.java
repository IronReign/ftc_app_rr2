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

public class scoringSystem {
    private int position = 0; //range of 1-1120
    private int speed = 0; //ticks per second
    private int backupSpeed = -1;
    DcMotor motorFlinger = null;
    DcMotor motorConveyor = null;
    private double power = 0;
    static int ticksPerRot = 1680;
    private double powerConveyor = 0;
    private long flingTimer = 0;

    public scoringSystem(DcMotor motorFlinger, DcMotor motorConveyor){
        position = 0;
        this.motorFlinger = motorFlinger;
        this.motorConveyor = motorConveyor;
        resetFlinger();
    }

    public scoringSystem(int ticksPerSec, DcMotor motorFlinger, DcMotor motorConveyor){
        position = 0;
        this.speed = ticksPerSec;
        this.motorFlinger = motorFlinger;
        this.motorConveyor = motorConveyor;
        resetFlinger();
    }

    public scoringSystem(int ticksPerSec, int position, DcMotor motorFlinger, DcMotor motorConveyor){
        this.speed = ticksPerSec;
        this.position = position;
        this.motorFlinger = motorFlinger;
        this.motorConveyor = motorConveyor;
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
        motorFlinger.setPower(1);
    }

    public void fling(){
        motorConveyor.setPower(-1);
        flingTimer = System.nanoTime() + 200000000;
        while(flingTimer > System.nanoTime()){ powerConveyor = -1; }
        motorConveyor.setPower(0);
        motorFlinger.setMaxSpeed(ticksPerRot);
        if(Math.abs(position) > Math.abs(motorFlinger.getCurrentPosition()) - 15            //prevents incrementing target pos
           && Math.abs(position) < Math.abs((motorFlinger.getCurrentPosition()) + 15)){     //if the flinger is stuck
            position += ticksPerRot * 1;
            runToPosition();
            while(Math.abs(position) > Math.abs(motorFlinger.getCurrentPosition()) - 10            //prevents overshooting
                    && Math.abs(position) < Math.abs((motorFlinger.getCurrentPosition()) + 10)){}
            flingTimer = System.nanoTime() + 500000000;
            while(flingTimer > System.nanoTime()){}
        }
        motorConveyor.setPower(0);
        flingTimer = System.nanoTime() + 500000000;
        while(flingTimer > System.nanoTime()){ powerConveyor = 0; }
        motorConveyor.setPower(1);
        flingTimer = System.nanoTime() + 1000000000;
        while(flingTimer > System.nanoTime()){ powerConveyor = 1; }
        powerConveyor = 0;
    }

    public void collect(){
        if (powerConveyor == 0)
            powerConveyor = 1;
        else
            powerConveyor = 0;
    }

    public void eject(){
        if (powerConveyor == 0)
            powerConveyor = -1;
        else
            powerConveyor = 0;
    }

    public void updateCollection(){
        motorConveyor.setPower(powerConveyor);
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
