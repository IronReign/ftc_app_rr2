package org.firstinspires.ftc.teamcode.localization;

/*
    Kinematics methods to help with localization
 */

import java.util.List;

public class TankKinematics {
    public static Pose2d wheelToRobotVelocity(List<Integer> wheelVelos, double trackWidth) {
        return new Pose2d(
                new Vector2d(
                (wheelVelos.get(0) + wheelVelos.get(1)) / 2.0,
                0.0),
                (-wheelVelos.get(0) + wheelVelos.get(1)) / trackWidth
        );
    }

}
