package org.firstinspires.ftc.teamcode.vision.dummy;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.vision.GoldPos;
import org.firstinspires.ftc.teamcode.vision.Viewpoint;
import org.firstinspires.ftc.teamcode.vision.VisionProvider;

public abstract class AbstractDummyVisionIntegration implements VisionProvider {

    @Override
    public void initializeVision(HardwareMap hardwareMap, Telemetry telemetry, boolean enableTelemetry, Viewpoint viewpoint) {

    }

    @Override
    public abstract GoldPos detect();

    @Override
    public void shutdownVision() {}

    @Override
    public void reset() {}

}
