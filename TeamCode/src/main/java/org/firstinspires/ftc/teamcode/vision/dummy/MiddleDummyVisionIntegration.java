package org.firstinspires.ftc.teamcode.vision.dummy;

import org.firstinspires.ftc.teamcode.vision.GoldPos;

public class MiddleDummyVisionIntegration extends AbstractDummyVisionIntegration {

    @Override
    public GoldPos detect() {
        return GoldPos.MIDDLE;
    }

}