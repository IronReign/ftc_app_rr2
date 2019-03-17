package org.firstinspires.ftc.teamcode.modernControl;

import org.apache.commons.math3.linear.RealMatrix;

/*
    Represents the system we are controlling
    Created by Ben on 3/13/19
 */

public class StateSpacePlant {
    RealMatrix A,B,C,D, X, Y;
    public StateSpacePlant(RealMatrix A, RealMatrix B, RealMatrix C, RealMatrix D) {
        this.A = A; // system matrix
        this.B = B; // input matrix
        this.C = C; // maps states to measurements
        this.D = D; // useless pls dont use
    }


    public void setX(RealMatrix newX) {
        X = newX;
    }

    public RealMatrix A() { return A; }
    public RealMatrix B() { return B; }
    public RealMatrix C() { return C; }
    public RealMatrix D() { return D; }
    public RealMatrix X() { return X; }
    public RealMatrix Y() { return Y; }

    public RealMatrix updateX(RealMatrix x, RealMatrix u) {
        return A().multiply(x).add(B().multiply(u));
    }

    public RealMatrix updateY(RealMatrix u) {
        return C().multiply(X()).add(D().multiply(u));
    }

    public void update(RealMatrix u) {
        X = updateX(X(), u);
        Y = updateY(u);
    }
}
