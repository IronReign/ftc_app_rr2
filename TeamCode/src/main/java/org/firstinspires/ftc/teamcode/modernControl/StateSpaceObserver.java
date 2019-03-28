package org.firstinspires.ftc.teamcode.modernControl;

import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

/*
    Used to estimate states not being directly measured
    Uses a kalman filter to reduce noise
    Created by Ben on 3/15/19
 */

public class StateSpaceObserver {
    RealMatrix K; // kalman gain
    StateSpacePlant plant;
    RealMatrix xHat; // current state estimate

    public StateSpaceObserver(RealMatrix K, StateSpacePlant plant) {
        this.K = K;
        this.plant = plant;
    }


    public void setXHat(RealMatrix xHat) {
        this.xHat = xHat;
    }
    public void setXHat(int i, double value) {
        xHat.setEntry(i, 0, value);
    }

    public RealMatrix XHat() { return xHat; }

    public RealMatrix K() { return K; }

    public double K(int i, int j) { return K.getEntry(i,j); }


    public double XHat(int i) { return XHat().getEntry(i,0); }

    public void predict(RealMatrix newU) {
        xHat = plant.updateX(XHat(), newU);
    }

    public void correct(RealMatrix u, RealMatrix y) {
        // correct xHat using a kalman filter
        RealMatrix matOne = y.subtract(plant.C().multiply(XHat()));
        RealMatrix matTwo = plant.D().multiply(u);
        xHat = xHat.add(K().multiply(matOne.subtract(matTwo)));
    }
}
