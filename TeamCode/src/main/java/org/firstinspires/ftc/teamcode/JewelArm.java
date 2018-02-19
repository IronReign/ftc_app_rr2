package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by 2938061 on 11/10/2017.
 */

public class JewelArm {

    private Servo servoJewelLeft;
    private Servo servoJewelRight;
    private Servo servoJewel;
    private ColorSensor colorJewel;
    int jewelStartPos = 950;
    int jewelUpPos = 1000;
    int jewelDownPos = 1925;
    private int jewelLeft = 900;
    private int jewelMid = 1500;
    private int jewelRight = 2100;
    public int jewelPos;

    public JewelArm(Servo servoJewelLeft, Servo servoJewelRight, ColorSensor colorJewel, Servo servoJewel){
        this.servoJewelLeft = servoJewelLeft;
        this.servoJewelRight = servoJewelRight;
        this.colorJewel = colorJewel;
        this.servoJewel = servoJewel;
    }

    public void startArm(){
        servoJewelLeft.setPosition(servoNormalize(jewelStartPos));
        servoJewelRight.setPosition(servoNormalize(jewelStartPos));
        jewelPos = jewelStartPos;
    }

    public void liftArm(){
        servoJewelLeft.setPosition(servoNormalize(jewelUpPos));
        servoJewelRight.setPosition(servoNormalize(jewelUpPos));
        jewelPos = jewelUpPos;
    }
    public void lowerArm(){
        jewelPos = jewelDownPos;
        servoJewelLeft.setPosition(servoNormalize(jewelDownPos));
        servoJewelRight.setPosition(servoNormalize(jewelDownPos));
    }

    public void hitLeft(){
        servoJewel.setPosition(servoNormalize(jewelLeft));
    }
    public void hitRight(){
        servoJewel.setPosition(servoNormalize(jewelRight));
    }
    public void center(){
        servoJewel.setPosition(servoNormalize(jewelMid));
    }

    public static double servoNormalize(int pulse){
        double normalized = (double)pulse;
        return (normalized - 750.0) / 1500.0; //convert mr servo controller pulse width to double on _0 - 1 scale
    }

}
