package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Abhijit on 11/7/18.
 */

public class ElbowSystem {

    DcMotor elbow = null;
    DcMotor intake = null;

    private int elbowFull = 4000;
    private int elbowStow = 2500; //stacking height
    private int elbowMin = 50;
    //private int liftAuto = 500;
   // private int liftAuto2 = 1500;
    private int liftPlanck = 0;


    public ElbowSystem(DcMotor elbow, DcMotor intake){
        this.elbow = elbow;
        this.intake=intake;
    }

    public void pickUp(){ intake.setPower(1);}
    public void eject(){ intake.setPower(-1);}
    public void stopIntake(){ intake.setPower(0);}

    public void stopElbow(){
        elbow.setPower(0);
    }

    public void extendElbow(){
        if(elbow.getCurrentPosition() < elbowFull) elbow.setPower(.5);
        else elbow.setPower(0);
    }
    public void retractElbow(){
        if(elbow.getCurrentPosition() > elbowMin) elbow.setPower(-.5);
        else elbow.setPower(0);
    }

    public void extendElbow2(){
        if (elbow.getCurrentPosition() < elbowFull && elbow.getTargetPosition() < elbowFull) {
            elbow.setTargetPosition((int) Math.min(elbow.getCurrentPosition() + liftPlanck, elbowFull));
            elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elbow.setPower(1);
        }
    }
    public void retractElbow2() {
        if (elbow.getCurrentPosition() > elbowMin && elbow.getTargetPosition() > elbowMin) {
            elbow.setTargetPosition((int) Math.max(elbow.getCurrentPosition() - liftPlanck, elbowMin));
            elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elbow.setPower(.8);
        }
    }

    public void goExtendMax() {

            elbow.setTargetPosition(elbowFull);
            elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elbow.setPower(1);

    }
    /**
    public void goLiftAuto() {

        elbow.setTargetPosition(liftAuto);
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbow.setPower(1);

    }

    public void goLiftAuto2() {

        elbow.setTargetPosition(liftAuto2);
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbow.setPower(1);

    }



    public void goLiftMin() {

        elbow.setTargetPosition(elbowMin);
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbow.setPower(1);

    }

    public void goLiftStack() {

        elbow.setTargetPosition(elbowStow);
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbow.setPower(1);

    }
    **/
    public int getMotorExtendPosition(){
        return elbow.getCurrentPosition();
    }

    public static double servoNormalize(int pulse){
        double normalized = (double)pulse;
        return (normalized - 750.0) / 1500.0; //convert mr servo controller pulse width to double on _0 - 1 scale
    }

}
