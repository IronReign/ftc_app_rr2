package org.firstinspires.ftc.teamcode.vision.dummy;

import org.firstinspires.ftc.teamcode.vision.GoldPos;

public class LeftDummyVisionIntegration extends AbstractDummyVisionIntegration {

    @Override
    public GoldPos detect() {
        return GoldPos.LEFT;
    }

}
