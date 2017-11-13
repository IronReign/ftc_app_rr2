package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by 2938061 on 11/10/2017.
 */

public class JewelArm {

    private Servo servoJewel;
    private NormalizedColorSensor colorJewel;
    int jewelUpPos = 850;
    int jewelDownPos = 2050;
    public int jewelPos;

    public JewelArm(Servo servoJewel, NormalizedColorSensor colorJewel){
        this.servoJewel = servoJewel;
        this.colorJewel = colorJewel;
    }

    public void liftArm(){
        servoJewel.setPosition(servoNormalize(jewelUpPos));
        jewelPos = jewelUpPos;
    }
    public void lowerArm(){
        jewelPos = jewelDownPos;
        servoJewel.setPosition(servoNormalize(jewelDownPos));
    }

    public static double servoNormalize(int pulse){
        double normalized = (double)pulse;
        return (normalized - 750.0) / 1500.0; //convert mr servo controller pulse width to double on _0 - 1 scale
    }

}
