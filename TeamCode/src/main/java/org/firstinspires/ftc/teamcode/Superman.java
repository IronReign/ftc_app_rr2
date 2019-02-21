package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
/**
 * Created by Abhijit on 11/7/18.
 */
public class Superman {
    DcMotor superman = null;
    int supermanPosInternal = 0;
    int supermanPos = 0;
    double supermanPwr = 0;
    //all filler values; need to be updated to reflect actual positions
    public int pos_Intake = 10;
    public int pos_Deposit = 354;
    public int pos_DepositPartial = 250;
    public int pos_Maximum = 500;
    public int pos_autonPrelatch = 400;
    public int pos_prelatch = 500;
    public int pos_latched = 500;
    public int pos_postlatch = 0;
    public int pos_stowed = 0;
    public int pos_driving = 0; //todo - experiment with driving with superman set around 100 (slightly angled) to see if it is more responsive - higher battery drain because superman is straining, but less actual downforce on omni


    //filler value; needs to be updated to reflect actual ratio
    public double ticksPerDegree = 5;
    public boolean active = true;

    public Superman(DcMotor superman) {
        superman.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        superman.setDirection(DcMotor.Direction.REVERSE);
        this.superman = superman;
    }

    public void update() {
        if (active && supermanPosInternal != supermanPos) { //don't keep updating if we are close to target position
            supermanPosInternal = supermanPos;
            superman.setTargetPosition(supermanPos);
            superman.setPower(supermanPwr);
        }
    }

    public boolean isActive() {
        return active;
    }

    public void setTargetPosition(int pos) {
        supermanPos = pos;
    }


    public boolean setTargetPosition(int pos, double speed) {
        setPower(speed);
        setTargetPosition(pos);
        if (nearTarget()) return true;
        else return false;
    }

    public int getTargetPosition() {
        return supermanPos;
    }

    public int getCurrentPosition() {
        return superman.getCurrentPosition();
    }

    public boolean nearTarget(){
        if ((Math.abs(getCurrentPosition()-getTargetPosition()))<15) return true;
        else return false;
    }

    public void setPower(double pwr) {
        supermanPwr = pwr;
    }

    public void kill() {
        setPower(0);
        update();
        active = false;
    }

    public void restart(double pwr) {
        setPower(pwr);
        active = true;
    }

    public void resetEncoders() {
        //just encoder - only safe to call if we know robot is in normal ground state
        //this should stop the motor
        //todo: maybe verify against imu - though that would assume that imu was calibrated correctly
        superman.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        superman.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public boolean reset(){
        //reset the superman arm by retracting it to the physical start position on very low power until it stops moving
        //this has to be called repeatedly until it returns true - unless we decide to start a dedicated thread
        //only safe to call if superman is free to move and is not hyper-extended - operator needs to verify this
        //todo: this is just a stub - need to decide if we need to implement

        return false;
        // start tucking in the arm
        //            superman.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //            superman.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //            superman.setPower(.1); // we assume this is low enough to move it until it jams without skipping gears

        //after stalling
        //            superman.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //            superman.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //            setTargetPosition (probably back to zero or minimum) - but this makes position control active again
    }

    public void raise() {
        setTargetPosition(Math.min(getCurrentPosition() + 30, pos_Maximum));
    }

    public void lower() {
        setTargetPosition(Math.max(getCurrentPosition() -30, 0));
    }

    public void runToAngle(double angle) {
        setTargetPosition((int) (angle * ticksPerDegree));
    }

    public static double servoNormalize(int pulse) {
        double normalized = (double) pulse;
        return (normalized - 750.0) / 1500.0; //convert mr servo controller pulse width to double on _0 - 1 scale
    }

}
