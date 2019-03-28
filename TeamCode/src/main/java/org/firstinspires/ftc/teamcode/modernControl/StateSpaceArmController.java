package org.firstinspires.ftc.teamcode.modernControl;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;

public class StateSpaceArmController {
    private RealMatrix y;
    StateSpaceLoop loop;
    private boolean atReferences = false;

    public StateSpaceArmController() {
        y = MatrixUtils.createRealMatrix(1,1);
    }

    // enable/disable the loop
    public void setReferences(double angle, double angularVelocity) {
        RealMatrix nextR = MatrixUtils.createRealMatrix(2,1);
        nextR.setEntry(1,1, angle);
        nextR.setEntry(2,1, angularVelocity);
        loop.setNextR(nextR);
    }

    public boolean atReferences() { return atReferences; }
    public void setMeasuredAngle(double angle) { y.setEntry(1,1, angle); }

    double controllerVoltage() { return loop.U(0); }

    double estimatedAngle() { return loop.XHat(0); }
    public void update() {
        loop.correct(y);
        loop.predict();
    }
}
