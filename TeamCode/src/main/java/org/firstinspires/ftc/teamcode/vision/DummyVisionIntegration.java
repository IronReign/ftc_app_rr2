package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DummyVisionIntegration implements VisionProvider {

    @Override
    public void initializeVision(HardwareMap hardwareMap, Telemetry telemetry, boolean enableTelemetry, Viewpoint viewpoint) {

    }

    @Override
    public GoldPos detect() {
        return GoldPos.MIDDLE;
    }

    @Override
    public void shutdownVision() {}

    @Override
    public void reset() {}

}
