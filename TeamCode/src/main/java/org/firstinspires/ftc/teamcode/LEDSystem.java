package org.firstinspires.ftc.teamcode;

/**
 * Created by Karim on 3/9/2018.
 */

import com.qualcomm.robotcore.hardware.Servo;


public class LEDSystem {
    Servo movement;

    public int off = 1200;
    public int aqua = 1300;
    public int blue = 1460;
    public int pink = 1595;
    public int red = 1780;

   public LEDSystem(Servo ledServo)
   {
       this.movement = ledServo;

   }

    long futureTime(float seconds){
        return System.nanoTime() + (long) (seconds * 1e9);
    }


    //servoJewelLeft.setPosition(servoNormalize(jewelStartPos));

    public void offPos(){
        movement.setPosition(servoNormalize(off));
    }
    public void aquaPos(){
        movement.setPosition(servoNormalize(aqua));
    }
    public void bluePos(){
        movement.setPosition(servoNormalize(blue));
    }
    public void pinkPos(){
        movement.setPosition(servoNormalize(pink));
    }
    public void redPos() {
        movement.setPosition(servoNormalize(red));
    }

    public static double servoNormalize(int pulse){
        double normalized = (double)pulse;
        return (normalized - 750.0) / 1500.0; //convert mr servo controller pulse width to double on _0 - 1 scale
    }


}
