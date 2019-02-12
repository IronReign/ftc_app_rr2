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
    public int pos_Deposit = 585;

    public int pos_prelatch = 217;
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

    public int getTargetPosition() {
        return supermanPos;
    }

    public int getCurrentPosition() {
        return superman.getCurrentPosition();
    }

    public boolean nearTarget(){
        if ((Math.abs( getCurrentPosition())-getTargetPosition())<15) return true;
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

    public void raise() {
        setTargetPosition(Math.min(getCurrentPosition() + 30, pos_Deposit));
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
