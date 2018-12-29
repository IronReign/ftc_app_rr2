package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OpenCVIntegration implements VisionProvider {

    public void initializeVision(HardwareMap hardwareMap, Telemetry telemetry) {

    }

    public void shutdownVision() {

    }

    public GoldPos detect() {
        return GoldPos.ERROR3;
    }
}
