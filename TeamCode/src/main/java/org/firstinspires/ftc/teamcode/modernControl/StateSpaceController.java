package org.firstinspires.ftc.teamcode.modernControl;

import org.apache.commons.math3.linear.RealMatrix;

public class StateSpaceController {
    private RealMatrix K, Kff, Umin, Umax, R, U;
    private StateSpacePlant plant;

    public StateSpaceController(RealMatrix K, RealMatrix Kff, RealMatrix Umin, RealMatrix Umax, StateSpacePlant plant) {
        this.K = K; // controller gain matrix (determined from lqr
        this.Kff = Kff; // feedforward
        this.Umin = Umin;
        this.Umax = Umax;
        this.plant = plant;
    }

    public RealMatrix K() { return K; }
    public RealMatrix Kff() { return Kff; }
    public RealMatrix R() { return R; }
    public RealMatrix U() { return U; }
    public double U(int i) { return U().getEntry(i,0); }

    public void update(RealMatrix x) {
       RealMatrix term1 = K().multiply(R.subtract(x));
       RealMatrix term2 = Kff().multiply(R.subtract(plant.A().multiply(R)));
       U = term1.add(term2);
       // cap u
    }

    public void update(RealMatrix x, RealMatrix nextR) {
        RealMatrix term1 = K().multiply(R.subtract(x));
        RealMatrix term2 = Kff().multiply(R.subtract(plant.A().multiply(R)));
        U = term1.add(term2);
        // cap u
        R = nextR;
    }

}
