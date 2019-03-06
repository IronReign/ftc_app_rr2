package org.firstinspires.ftc.teamcode;

/**
 * Created by Karim on 3/9/2018.
 */

import com.qualcomm.robotcore.hardware.Servo;


public class LEDSystem {
    Servo movement;

    public enum Color {
        RED(1418), BLUE(1425), GOLD(1595), STRESS(1079), CALM(1108), GAME_OVER(1208), PURPLE(1787), OFF(1200);

        Color(int pos) {
            this.pos = pos;
        }

        public final int pos;
    }

    public LEDSystem(Servo ledServo) {
        this.movement = ledServo;
    }

    public void setColor(Color color) {
        movement.setPosition(servoNormalize(color.pos));
    }

    public static double servoNormalize(int pulse) {
        double normalized = (double) pulse;
        return (normalized - 750.0) / 1500.0; //convert mr servo controller pulse width to double on _0 - 1 scale
    }

}
