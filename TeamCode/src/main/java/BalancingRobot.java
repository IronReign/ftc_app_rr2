import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.PoseBigWheel;

public class BalancingRobot {
    PoseBigWheel robot;
    OpMode opMode;


    public BalancingRobot(PoseBigWheel robot) {
        this.robot = robot;
    }

    public BalancingRobot(PoseBigWheel robot, OpMode opMode) { // we pass in the opmode in case we want to enable telemtry tuning

    }
}
