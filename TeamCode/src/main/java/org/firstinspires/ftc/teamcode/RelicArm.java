package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Craniumski on 11/10/2017.
 */

public class RelicArm {

    private Servo shoulder;
    private Servo elbow;
    private Servo grip;

    private long extendTimer = 0;
    private float extendDuration = 5.5f;
    public boolean autoExtend = false;

    int shoulderExtend = 2200;
    int shoulderStop = 1500;
    int shoulderRetract = 900;
    int elbowTucked = 1050;
    int elbowApproach = 2125;
    int elbowGrab = 2175;
    int gripOpen = 2125;
    int gripClosed = 825;

    public int elbowTarget = elbowTucked;

    public RelicArm(Servo shoulder, Servo elbow, Servo grip){
        this.shoulder = shoulder;
        this.elbow = elbow;
        this.grip = grip;
    }



    public void autonomousExtend(){
        if(!autoExtend){
            shoulder.setPosition(servoNormalize(shoulderExtend));
            autoExtend = true;
            extendTimer = futureTime(extendDuration);
        }
        if(extendTimer > System.nanoTime())
        {
            stopShoulder();
            autoExtend = false;
        }
    }

    public void extend(){
        autoExtend = false;
        shoulder.setPosition(servoNormalize(shoulderExtend));
    }

    public void stopShoulder(){
        autoExtend = false;
        shoulder.setPosition(servoNormalize(shoulderStop));
    }

    public void retract(){
        autoExtend = false;
        shoulder.setPosition(servoNormalize(shoulderRetract));
    }

    public void deployElbow(){
        if(elbow.getPosition() > servoNormalize(elbowApproach) - .03 && elbow.getPosition() < servoNormalize(elbowApproach) + .03){
//            elbow.setPosition(servoNormalize(elbowGrab));
            elbowTarget = elbowGrab;
        }
        else{
//            elbow.setPosition(servoNormalize(elbowApproach));
            elbowTarget = elbowApproach;
        }
    }

    public void tuckElbow(){
//        elbow.setPosition(servoNormalize(elbowTucked));
        elbowTarget = elbowTucked;
    }

    public void openGrip(){
        grip.setPosition(servoNormalize(gripOpen));
    }

    public void closeGrip(){
        grip.setPosition(servoNormalize(gripClosed));
    }

    public void toggleGrip(){
        if(grip.getPosition() > servoNormalize(gripOpen) - .05 && grip.getPosition() < servoNormalize(gripOpen) + .05){
            closeGrip();
        }
        else{
            openGrip();
        }
    }

    public void setElbow(int pwm){
        elbowTarget = pwm;
    }

    public void setGrip(int pwm){
        grip.setPosition(servoNormalize(pwm));
    }

    public void update(){
        elbow.setPosition(servoNormalize(elbowTarget));
    }



    public static double servoNormalize(int pulse){
        double normalized = (double)pulse;
        return (normalized - 750.0) / 1500.0; //convert mr servo controller pulse width to double on _0 - 1 scale
    }

    long futureTime(float seconds){
        return System.nanoTime() + (long) (seconds * 1e9);
    }

}
