package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by tycho on 10/15/2017.
 */

public class PickAndPlace {

    DcMotor motorLift = null;
    Servo servoGrip = null;

    private int liftMax = 4000;
    private int liftStack = 2500; //stacking height
    private int liftMin = 50;
    private int liftPlanck = 450; //smallest distance to increment lift by when using runToPosition

    boolean gripOpen = false;
    int gripOpenPos = 900;
    int gripClosedPos = 2110;

    public PickAndPlace(DcMotor motorLift, Servo servoGrip){
        this.motorLift = motorLift;
        this.servoGrip = servoGrip;
    }

    public void ToggleGrip (){
        if (gripOpen) {
            gripOpen = false;
            servoGrip.setPosition(ServoNormalize(gripClosedPos));
        }
        else {
            gripOpen = true;
            servoGrip.setPosition(ServoNormalize(gripOpenPos));
        }
    }


    public void stopLift(){
        motorLift.setPower(0);
    }

    public void raiseLift(){
        if(motorLift.getCurrentPosition() < liftMax) motorLift.setPower(.5);
        else motorLift.setPower(0);
    }
    public void lowerLift(){
        if(motorLift.getCurrentPosition() > liftMin) motorLift.setPower(-.5);
        else motorLift.setPower(0);
    }

    public void raiseLift2(){
        if (motorLift.getCurrentPosition() < liftMax && motorLift.getTargetPosition() < liftMax) {
            motorLift.setTargetPosition((int) Math.min(motorLift.getCurrentPosition() + liftPlanck, liftMax));
            motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLift.setPower(1);
        }
    }
    public void lowerLift2() {
        if (motorLift.getCurrentPosition() > liftMin && motorLift.getTargetPosition() > liftMin) {
            motorLift.setTargetPosition((int) Math.max(motorLift.getCurrentPosition() - liftPlanck, liftMin));
            motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLift.setPower(.8);
        }
    }
    public void goLiftMax() {

            motorLift.setTargetPosition(liftMax);
            motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLift.setPower(1);

    }

    public void goLiftMin() {

        motorLift.setTargetPosition(liftMin);
        motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLift.setPower(1);

    }

    public void goLiftStack() {

        motorLift.setTargetPosition(liftStack);
        motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLift.setPower(1);

    }

    public int getMotorLiftPosition(){
        return motorLift.getCurrentPosition();
    }

    public static double ServoNormalize(int pulse){
        double normalized = (double)pulse;
        return (normalized - 750.0) / 1500.0; //convert mr servo controller pulse width to double on _0 - 1 scale
    }

}
