package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by tycho on 11/18/2016.
 */

public class ParticleSystem {
    DcMotor motorLauncher = null;
    DcMotor motorConveyor = null;
    Servo servoGate       = null;

    private int position         = 0; //range of 1-1120
    private int speed            = 0; //ticks per second
    private int backupSpeed      = -1;
    private double powerLauncher = 0;
    private double powerConveyor = 0;
    static int ticksPerRot       = 1680;
    private long flingTimer      = 0;

    private double gateClosed    = ServoNormalize(2150);
    private double gateOpen      = ServoNormalize(1150);
    private double servoPosition = gateClosed;
    private double collectPower  = .65;
    private double ejectPower    = -.65;
    private double launchPower   = 1;

    public ParticleSystem(DcMotor motorLauncher, DcMotor motorConveyor, Servo servoGate){
        this.position      = 0;
        this.motorLauncher = motorLauncher;
        this.motorConveyor = motorConveyor;
        this.servoGate     = servoGate;
        resetFlinger();
    }

    public ParticleSystem(int ticksPerSec, DcMotor motorLauncher, DcMotor motorConveyor, Servo servoGate){
        this.position      = 0;
        this.speed         = ticksPerSec;
        this.motorLauncher = motorLauncher;
        this.motorConveyor = motorConveyor;
        this.servoGate     = servoGate;
        resetFlinger();
    }

    public ParticleSystem(int ticksPerSec, int position, DcMotor motorLauncher, DcMotor motorConveyor, Servo servoGate){
        this.speed         = ticksPerSec;
        this.position      = position;
        this.motorLauncher = motorLauncher;
        this.motorConveyor = motorConveyor;
        this.servoGate     = servoGate;
        resetFlinger();
    }

    public void setPosition(int position){
        this.position = position;
        runToPosition();
    }

    public void setSpeed(int ticksPerSec){
        this.speed = ticksPerSec;
        this.motorLauncher.setMaxSpeed(speed);
    }

    public void runToPosition(){
        this.motorLauncher.setTargetPosition(position);
    }

    public void halfCycle(){
        resetFlinger();
        setPosition(ticksPerRot/2);
    }

    public void resetFlinger(){
        motorLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLauncher.setMaxSpeed(speed);
        setPosition(0);
        motorLauncher.setPower(0);
    }

    public void fling(){
        motorConveyor.setPower(-powerLauncher);
        flingTimer = System.nanoTime() + 200000000;
        while(flingTimer > System.nanoTime()){ powerConveyor = -1; }
        motorConveyor.setPower(0);
        motorLauncher.setMaxSpeed(ticksPerRot);
        if(Math.abs(position) > Math.abs(motorLauncher.getCurrentPosition()) - 15            //prevents incrementing target pos
           && Math.abs(position) < Math.abs((motorLauncher.getCurrentPosition()) + 15)){     //if the flinger is stuck
            position += ticksPerRot * 1;
            runToPosition();
            while(Math.abs(position) > Math.abs(motorLauncher.getCurrentPosition()) - 10            //prevents overshooting
                    && Math.abs(position) < Math.abs((motorLauncher.getCurrentPosition()) + 10)){}
            flingTimer = System.nanoTime() + 500000000;
            while(flingTimer > System.nanoTime()){}
        }
        motorConveyor.setPower(0);
        flingTimer = System.nanoTime() + 500000000;
        while(flingTimer > System.nanoTime()){ powerConveyor = 0; }
        motorConveyor.setPower(powerLauncher);
        flingTimer = System.nanoTime() + 1000000000;
        while(flingTimer > System.nanoTime()){ powerConveyor = powerLauncher; }
        powerConveyor = 0;
    }

    public void spinUp(){
        if(powerLauncher == 0)
            powerLauncher = launchPower;
        else
            powerLauncher = 0;
    }

    public void collect(){
        if (powerConveyor == 0)
            powerConveyor = collectPower;
        else
            powerConveyor = 0;
    }

    public void eject(){
        if (powerConveyor == 0)
            powerConveyor = ejectPower;
        else
            powerConveyor = 0;
    }

    public void launch(){
        if(servoPosition == gateClosed) {
            servoPosition = gateOpen;
            powerConveyor = collectPower/6;
        }
        else{
            servoPosition = gateClosed;
            powerConveyor = 0;
         }
    }

    public void updateCollection(){
        motorLauncher.setPower(powerLauncher);
        motorConveyor.setPower(powerConveyor);
        servoGate.setPosition(servoPosition);
    }

    public void emergencyStop(){
        backupSpeed = speed;
        motorLauncher.setPower(0);
        powerConveyor = 0;
        motorConveyor.setPower(0);
        setSpeed(0);
    }

    public boolean isStopped(){
        return backupSpeed >= 0;
    }

    public void restart(){
        setSpeed(backupSpeed);
        motorLauncher.setPower(1);
        backupSpeed = -1;
    }

    public double ServoNormalize(int pulse){
        double normalized = (double)pulse;
        return (normalized - 750.0) / 1500.0; //convert mr servo controller pulse width to double on _0 - 1 scale
    }
}
