package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface VisionProvider {

    public void initializeVision(HardwareMap hardwareMap, Telemetry telemetry, boolean enableTelemetry);

    public void shutdownVision();

    public GoldPos detect();

}
