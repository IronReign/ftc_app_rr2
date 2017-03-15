package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.Pose.ticksPerRot;

/**
 * Created by tycho on 11/18/2016.
 */

public class ParticleSystem {
    DcMotor motorLauncher = null;
    DcMotor motorConveyor = null;
    Servo   servoGate     = null;

    byte[] ballColorCache          = null;
    I2cDevice ballColorSensor      = null;
    I2cDeviceSynch ballColorReader = null;
    long ballColor                 = 0;

    private int position           = 0; //range of 1-1120
    private int speed              = 0; //ticks per second
    private int backupSpeed        = -1;
    private double powerLauncher   = 0;
    private int speedConveyor      = 0;
    private int loadSpeed          = 400;
    private int collectSpeed       = 3500;

    private long flingTimer        = 0;
    private long ejectTimer        = 0;
    private int launchState        = 0;
    private long prevTime          = 0;
    public  float flywheelSpeed    = 0;
    private long prevFlywheelTicks = 0;
    private boolean isBlue         = true;
    private boolean shouldEject    = false;
    private float minFlywheelSpeed = 750;

    private double gateClosed    = ServoNormalize(2150);
    private double gateOpen      = ServoNormalize(1150);
    private double servoPosition = gateClosed;
    private double powerConveyor  = 1.0;
    private double launchPower   = .8  ;

    public ParticleSystem(DcMotor motorLauncher, DcMotor motorConveyor, Servo servoGate, I2cDevice ballColorSensor, boolean isBlue){
        this.position        = 0;
        this.motorLauncher   = motorLauncher;
        this.motorConveyor   = motorConveyor;
        this.servoGate       = servoGate;
        this.ballColorSensor = ballColorSensor;
        this.isBlue          = isBlue;
        ballColorReader      = new I2cDeviceSynchImpl(ballColorSensor, I2cAddr.create8bit(0x64), false);
        ballColorReader.engage();
        ballColorReader.write8(3, 0);    //Set the mode of the color sensor using LEDState (0 = active, 1 = passive)
        motorConveyor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorConveyor.setMaxSpeed(4000);
        resetFlinger();
    }

    public ParticleSystem(int ticksPerSec, DcMotor motorLauncher, DcMotor motorConveyor, Servo servoGate, I2cDevice ballColorSensor, boolean isBlue){
        this.position        = 0;
        this.speed           = ticksPerSec;
        this.motorLauncher   = motorLauncher;
        this.motorConveyor   = motorConveyor;
        this.servoGate       = servoGate;
        this.ballColorSensor = ballColorSensor;
        this.isBlue          = isBlue;
        ballColorReader      = new I2cDeviceSynchImpl(ballColorSensor, I2cAddr.create8bit(0x64), false);
        ballColorReader.engage();
        ballColorReader.write8(3, 0);    //Set the mode of the color sensor using LEDState (0 = active, 1 = passive)
        motorConveyor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorConveyor.setMaxSpeed(4000);
        resetFlinger();
    }

    public ParticleSystem(int ticksPerSec, int position, DcMotor motorLauncher, DcMotor motorConveyor, Servo servoGate, I2cDevice ballColorSensor, boolean isBlue){
        this.speed           = ticksPerSec;
        this.position        = position;
        this.motorLauncher   = motorLauncher;
        this.motorConveyor   = motorConveyor;
        this.servoGate       = servoGate;
        this.ballColorSensor = ballColorSensor;
        this.isBlue          = isBlue;
        ballColorReader      = new I2cDeviceSynchImpl(ballColorSensor, I2cAddr.create8bit(0x64), false);
        ballColorReader.engage();
        ballColorReader.write8(3, 0);    //Set the mode of the color sensor using LEDState (0 = active, 1 = passive)
        motorConveyor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorConveyor.setMaxSpeed(4000);
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

    public boolean ballWrongColor(){
        if(isBlue){
            return (ballColor > 9 && ballColor < 12);
        }
        return ballColor == 3;
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
        while(flingTimer > System.nanoTime()){ speedConveyor = -1; }
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
        while(flingTimer > System.nanoTime()){ speedConveyor = 0; }
        motorConveyor.setPower(powerLauncher);
        flingTimer = System.nanoTime() + 1000000000;
        while(flingTimer > System.nanoTime()){ speedConveyor = collectSpeed; }
        speedConveyor = 0;
    }

    public void spinUpToggle(){
        if(powerLauncher == 0)
            spinUp();
        else
            spinDown();
    }

    public void spinUp(){
            powerLauncher = launchPower;
            }

    public void spinDown(){
        powerLauncher = 0;
    }

    public void collectToggle() {
        if (speedConveyor == 0){
            collectStart();
        }
        else {
            servoPosition = gateClosed;
            collectStop();
        }
    }

    public void collectStart(){
        servoPosition = gateClosed;
        speedConveyor = collectSpeed;
        powerConveyor = 1;
    }

    public void collectStop(){
        speedConveyor=0;

    }

    public void eject(){
        if (speedConveyor == 0) {
            servoPosition = gateClosed;
            speedConveyor = collectSpeed;
            powerConveyor = -1;
        }

        else {
            servoPosition = gateClosed;
            speedConveyor = 0;
            powerConveyor = 1;
        }
    }

    public void launchToggle() {
        if(flywheelSpeed > minFlywheelSpeed) {
            if (servoPosition == gateClosed) {
                launchBegin();
            } else {
                launchEnd();
            }
        }
        else{
            spinUp();
        }
    }

    public void launchBegin(){
        servoPosition = gateOpen;
        speedConveyor = loadSpeed;
    }

    public void launchEnd(){
        servoPosition = gateClosed;
        speedConveyor = 0;
    }

//        switch(launchState){
//            case 0:
//
//                break;
//            default:
//
//        }
//        if(flywheelSpeed > 800){
//            speedConveyor = collectPower / 2;
//        }
//        else speedConveyor = 0;


    public void toggleGate(boolean open){
        if(open)
            servoPosition = gateOpen;
        else
            servoPosition = gateClosed;
    }
    public void stopConveyor(){
        speedConveyor = 0;
    }
    public void setConveyorMode(DcMotor.RunMode mode){
        motorConveyor.setMode(mode);
    }

    public void updateCollection(){
        if(ballWrongColor() && !shouldEject){
            eject();
            ejectTimer = System.nanoTime() + (long) 2e9;
            shouldEject = true;
            motorConveyor.setPower(powerConveyor);
            motorConveyor.setMaxSpeed(speedConveyor);
        }
        else if(System.nanoTime() > ejectTimer && shouldEject){
            stopConveyor();
            shouldEject = false;
            motorConveyor.setPower(powerConveyor);
            motorConveyor.setMaxSpeed(speedConveyor);
        }
        else if(System.nanoTime() > ejectTimer && !shouldEject){
            motorConveyor.setPower(powerConveyor);
            motorConveyor.setMaxSpeed(speedConveyor);
        }
        motorLauncher.setPower(powerLauncher);
        servoGate.setPosition(servoPosition);
        ballColorCache = ballColorReader.read(0x04, 1);
        ballColor = (ballColorCache[0] & 0xFF);
        flywheelSpeed = (float)(motorLauncher.getCurrentPosition() - prevFlywheelTicks)/(((System.nanoTime() - prevTime)/(float)(1e9))); //returns ticks per second
        prevFlywheelTicks = motorLauncher.getCurrentPosition();
        prevTime = System.nanoTime();
    }

    public void emergencyStop(){
        backupSpeed = speed;
        motorLauncher.setPower(0);
        speedConveyor = 0;
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

    public long getBallColor() {
        return ballColor;
    }

    public double ServoNormalize(int pulse){
        double normalized = (double)pulse;
        return (normalized - 750.0) / 1500.0; //convert mr servo controller pulse width to double on _0 - 1 scale
    }
}
