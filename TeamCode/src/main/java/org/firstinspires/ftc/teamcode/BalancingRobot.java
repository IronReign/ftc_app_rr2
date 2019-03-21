package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.PoseBigWheel;
import org.firstinspires.ftc.teamcode.util.FixedCache;

public class BalancingRobot {
    private static final int CACHE_SIZE = 10;
    private PoseBigWheel robot;
    private FixedCache<Double> rollCache = new FixedCache<>(CACHE_SIZE);

    private double target;
    public BalancingRobot(PoseBigWheel robot, double target) {
        this.robot = robot;
        this.target = target;
    }

    public void setTarget(double newTarget) {
        this.target = newTarget;
    }

    public void update() {
        rollCache.add(robot.getRoll());
        if (rollCache.size() < 10)
            robot.balance(target);
        else
            robot.balance(getRollCacheAvg());
    }

    private double getRollCacheAvg() {
        double sum = 0;
        for (Double roll : rollCache)
            sum += roll;
        return sum / rollCache.size();
    }


}
