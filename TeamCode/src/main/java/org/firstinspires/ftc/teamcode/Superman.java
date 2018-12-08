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
    public int posIntake = 10;
    public int posDeposit = 585;
    public int posPreLatch = 10;
    public int posLatch = 297;
    public int posPostLatch=10;
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

    public void advance() {
        if(getCurrentPosition()<500)
            setTargetPosition(getCurrentPosition() + 30);
    }

    public void retreat() {
        if(getCurrentPosition()>-30)
            setTargetPosition(getCurrentPosition() - 30);
    }

    public void runToAngle(double angle) {
        setTargetPosition((int) (angle * ticksPerDegree));
    }

    public static double servoNormalize(int pulse) {
        double normalized = (double) pulse;
        return (normalized - 750.0) / 1500.0; //convert mr servo controller pulse width to double on _0 - 1 scale
    }

}
