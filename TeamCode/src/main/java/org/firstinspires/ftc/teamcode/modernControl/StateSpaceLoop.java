package org.firstinspires.ftc.teamcode.modernControl;

import org.apache.commons.math3.linear.RealMatrix;

public class StateSpaceLoop {
    StateSpaceController controller;
    StateSpacePlant plant;
    StateSpaceObserver observer;
    RealMatrix nextR;

    public StateSpaceLoop(StateSpaceController controller, StateSpacePlant plant, StateSpaceObserver  observer) {
        this.controller = controller;
        this.plant = plant;
        this.observer = observer;
    }

    public RealMatrix XHat() {
        return observer.XHat();
    }

    public void setXHat(RealMatrix xHat) {
        observer.setXHat(xHat);
    }

    public RealMatrix U() { return controller.U(); }
    public double U(int i) { return controller.U(i); }

    public void setNextR(RealMatrix r) {
        nextR = r;
    }

    public StateSpacePlant getPlant() { return plant; }

    public StateSpaceController getController() {
        return controller;
    }

    // impl a reset method

    public RealMatrix error() { return controller.R().subtract(observer.XHat()); }
    public void correct(RealMatrix y) { observer.correct(controller.U(), y);}
    public void predict() {
        controller.update(observer.XHat(), nextR);
        observer.predict(controller.U());
    }
}
