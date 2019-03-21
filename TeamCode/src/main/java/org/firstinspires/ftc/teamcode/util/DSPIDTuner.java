package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Consumer;
import org.firstinspires.ftc.robotcore.external.Function;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PoseBigWheel;

/*
    Trash PID tuner avoid using
    ftc dashboard broke so this is used
 */

public class DSPIDTuner {
    private boolean[] buttonSavedStates = new boolean[6];
    private int dpad_down = 0;
    private int dpad_up = 1;
    private int button_a = 2;
    private int button_y = 3;
    private int right_bumper = 4;
    private int left_bumper = 5;

    private Gamepad gamepad;
    private Telemetry telemetry; // make an option to use telemetry

    private double kP, kI, kD, target, error;
    private double pInterval = 0.05;
    private double iInterval = 1;
    private double dInterval = 0.02;
    private double targetInterval = 1;
    private Double[] gains = new Double[3];

    PoseBigWheel robot;
    Consumer<Double[]> updateGains;

    public DSPIDTuner(Gamepad gamepad, Telemetry telemetry, Consumer<Double[]> updateGains) {
        this.gamepad = gamepad;
        this.telemetry = telemetry;
        this.updateGains = updateGains;
    }

    public DSPIDTuner(Gamepad gamepad, Telemetry telemetry, double kP, double kI, double kD, Consumer<Double[]> updateGains) {
        this.gamepad = gamepad;
        this.telemetry = telemetry;
        this.updateGains = updateGains;
        this.kP = kP;
        this.kD = kD;
        this.kI = kI;
    }

    public void setTarget(double target) {
        this.target = target;
    }

    public void setpInterval(double interval) {
        pInterval = interval;
    }

    public void setiInterval(double interval) {
        iInterval = interval;
    }

    public void setdInterval(double interval) {
        dInterval = interval;
    }

    public void setTargetinterval(double interval) {
        targetInterval = interval;
    }

    public void setP(double kP) {
        this.kP = kP;
    }

    public void setI(double kI) {
        this.kI  = kI;
    }

    public void setD(double kD) {
        this.kD = kD;
    }


    public void update(double current) {
        error = target - current;
        if (toggleAllowed(gamepad.y, button_y))
            kP += pInterval;
        else if (toggleAllowed(gamepad.a, button_a))
            kP -= pInterval;
        if (toggleAllowed(gamepad.dpad_down, dpad_down))
            kD -= dInterval;
        else if (toggleAllowed(gamepad.dpad_up, dpad_up))
            kD += dInterval;
        if (toggleAllowed(gamepad.right_bumper,right_bumper))
            target += targetInterval;
        else if (toggleAllowed(gamepad.left_bumper, left_bumper))
            target -= targetInterval;
        gains[0] = kP;
        gains[1] = kI;
        gains[2] = kD;
        updateGains.accept(gains);

        telemetry.addData("P Coeff: ", kP);
        telemetry.addData("D Coeff: ", kD);
        telemetry.addData("Target Ange: ", target);
        telemetry.addData("Error: ", error);
        telemetry.update();

    }

    private boolean toggleAllowed(boolean button, int buttonIndex) {
        if (button) {
            if (!buttonSavedStates[buttonIndex]) { //we just pushed the button, and when we last looked at it, it was not pressed
                buttonSavedStates[buttonIndex] = true;
                return true;
            } else { //the button is pressed, but it was last time too - so ignore

                return false;
            }
        }

        buttonSavedStates[buttonIndex] = false; //not pressed, so remember that it is not
        return false; //not pressed

    }
}
