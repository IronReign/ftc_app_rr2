package org.firstinspires.ftc.teamcode.modernControl;


import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;

public class StateSpaceController {
    private RealMatrix K, Kff, Umin, Umax, R, U;
    private StateSpacePlant plant;
    private boolean enabled = false;
    private int numInputs;

    public StateSpaceController(RealMatrix K, RealMatrix Kff, RealMatrix Umin, RealMatrix Umax, StateSpacePlant plant, int numInputs) {
        this.K = K; // controller gain matrix (determined from lqr
        this.Kff = Kff; // feedforward
        this.Umin = Umin;
        this.Umax = Umax;
        this.plant = plant;
        this.numInputs = numInputs;
        U = MatrixUtils.createRealMatrix(numInputs, 1);
        R = MatrixUtils.createRealMatrix(numInputs, 1);
    }

    public void enable() { enabled = true; }

    public void disabled() {
        double[] zero = new double[numInputs];
        U.setColumn(0, zero);
        enabled = false;
    }

    public RealMatrix K() { return K; }
    public double K(int i, int j) { return K().getEntry(i, j); }


    public RealMatrix Kff() { return Kff; }
    public double Kff(int i, int j) { return Kff().getEntry(i,j); }

    public RealMatrix R() { return R; }
    public double R(int i) { return R().getEntry(i,0); }

    public RealMatrix U() { return U; }
    public double U(int i) { return U().getEntry(i,0); }

    public void reset() {
        double[] zero = new double[numInputs];
        U.setColumn(0, zero);
        R.setColumn(0, zero);
    }


    public void update(RealMatrix x) {
        if (enabled) {
            RealMatrix term1 = K().multiply(R.subtract(x));
            RealMatrix term2 = Kff().multiply(R.subtract(plant.A().multiply(R)));
            U = term1.add(term2);
            capU();
        }
    }

    public void update(RealMatrix x, RealMatrix nextR) {
        RealMatrix term1 = K().multiply(R.subtract(x));
        RealMatrix term2 = Kff().multiply(R.subtract(plant.A().multiply(R)));
        U = term1.add(term2);
        capU();
        R = nextR;
    }

    private void capU() {
        for (int i = 0; i < numInputs; i++) {
            if (U(i) > Umax.getEntry(i,0))
                U().setEntry(i,0, Umax.getEntry(i,0));
            else if (U(i) < Umin.getEntry(i,0))
                U().setEntry(i, 0, Umin.getEntry(i,0));
        }
    }



}
