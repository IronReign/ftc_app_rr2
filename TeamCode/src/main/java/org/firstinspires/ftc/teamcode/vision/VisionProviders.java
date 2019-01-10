package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.vision.dogecv.DogeCVIntegration;

public final class VisionProviders {
    private VisionProviders() { throw new RuntimeException("Utility Class"); }

    public static final Class<? extends VisionProvider>[] visionProviders =
            new Class[]{DogeCVIntegration.class, OpenCVIntegration.class, TensorflowIntegration.class , DummyVisionIntegration.class};

    public static final VisionProvider defaultVisionProvider() {
        return new DummyVisionIntegration();
    }
}
