package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Abhijit on 11/7/18.
 */

public class SupermanSystem {

    DcMotor superman = null;

    private int liftMax = 4000;
    private int liftMid = 2500; //stacking height
    private int liftMin = 50;
    private int liftAuto = 500;
    private int liftAuto2 = 1500;
    private int liftPlanck = 0;


    public SupermanSystem(DcMotor superman){
        this.superman = superman;
    }

    public void stopLift(){
        superman.setPower(0);
    }

    public void raiseLift(){
        if(superman.getCurrentPosition() < liftMax) superman.setPower(.5);
        else superman.setPower(0);
    }
    public void lowerLift(){
        if(superman.getCurrentPosition() > liftMin) superman.setPower(-.5);
        else superman.setPower(0);
    }

    public void raiseLift2(){
        if (superman.getCurrentPosition() < liftMax && superman.getTargetPosition() < liftMax) {
            superman.setTargetPosition((int) Math.min(superman.getCurrentPosition() + liftPlanck, liftMax));
            superman.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            superman.setPower(1);
        }
    }
    public void lowerLift2() {
        if (superman.getCurrentPosition() > liftMin && superman.getTargetPosition() > liftMin) {
            superman.setTargetPosition((int) Math.max(superman.getCurrentPosition() - liftPlanck, liftMin));
            superman.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            superman.setPower(.8);
        }
    }

    public void goLiftMax() {

            superman.setTargetPosition(liftMax);
            superman.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            superman.setPower(1);

    }

    public void goLiftAuto() {

        superman.setTargetPosition(liftAuto);
        superman.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        superman.setPower(1);

    }

    public void goLiftAuto2() {

        superman.setTargetPosition(liftAuto2);
        superman.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        superman.setPower(1);

    }



    public void goLiftMin() {

        superman.setTargetPosition(liftMin);
        superman.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        superman.setPower(1);

    }

    public void goLiftStack() {

        superman.setTargetPosition(liftMid);
        superman.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        superman.setPower(1);

    }

    public int getMotorLiftPosition(){
        return superman.getCurrentPosition();
    }

    public static double servoNormalize(int pulse){
        double normalized = (double)pulse;
        return (normalized - 750.0) / 1500.0; //convert mr servo controller pulse width to double on _0 - 1 scale
    }

}
