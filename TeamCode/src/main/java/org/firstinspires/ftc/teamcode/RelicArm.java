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

    int shoulderTucked = 1500;
    int shoulderMid = 1500;
    int shoulderDeployed = 1500;
    int elbowTucked = 1500;
    int elbowMid = 1500;
    int gripOpen = 1500;
    int gripClosed = 1500;

    public RelicArm(Servo shoulder, Servo elbow, Servo grip){
        this.shoulder = shoulder;
        this.elbow = elbow;
        this.grip = grip;
    }

    public void extend(){

    }

    public void retract(){

    }

    public void deployElbow(){

    }

    public void tuckElbow(){

    }

    public void openGrip(){

    }

    public void closeGrip(){

    }

    public void setElbow(int pwm){
        elbow.setPosition(servoNormalize(pwm));
    }

    public void setGrip(int pwm){
        grip.setPosition(servoNormalize(pwm));
    }



    public static double servoNormalize(int pulse){
        double normalized = (double)pulse;
        return (normalized - 750.0) / 1500.0; //convert mr servo controller pulse width to double on _0 - 1 scale
    }

}
