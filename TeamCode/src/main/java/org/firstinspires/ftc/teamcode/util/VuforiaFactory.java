package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.RC;

public class VuforiaFactory {

    private static VuforiaLocalizer vuforiaLocalizer;

    public enum Robot {
        BigWheel,
        Mini_Mech,
        Kraken
    }

    public static void init(Robot robot) {
        if (robot == Robot.BigWheel)
            vuforiaLocalizer = getBigWheelVuforiaLocalizer();
    }

    public static VuforiaLocalizer getVuforiaLocalizer() { return vuforiaLocalizer; }

    private static VuforiaLocalizer getBigWheelVuforiaLocalizer() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = RC.VUFORIA_LICENSE_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        return ClassFactory.getInstance().createVuforia(parameters);
    }


}
