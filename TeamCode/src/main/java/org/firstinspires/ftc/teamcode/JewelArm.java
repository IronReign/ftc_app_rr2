package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by 2938061 on 11/10/2017.
 */

public class JewelArm {

    private Servo servoJewel;
    private NormalizedColorSensor colorJewel;
    private int jewelUpPos;
    private int jewelDownPos;
    public int jewelPos;

    public JewelArm(Servo servoJewel, NormalizedColorSensor colorJewel, int jewelUpPos, int jewelDownPos){
        this.servoJewel = servoJewel;
        this.colorJewel = colorJewel;
        this.jewelUpPos = jewelUpPos;
        this.jewelDownPos = jewelDownPos;
    }

    public void liftArm(){
        servoJewel.setPosition(ServoNormalize(jewelUpPos));
        jewelPos = jewelUpPos;
    }
    public void lowerArm(){
        jewelPos = jewelDownPos;
        servoJewel.setPosition(ServoNormalize(jewelDownPos));
    }

    public static double ServoNormalize(int pulse){
        double normalized = (double)pulse;
        return (normalized - 750.0) / 1500.0; //convert mr servo controller pulse width to double on _0 - 1 scale
    }

}
