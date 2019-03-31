package org.firstinspires.ftc.teamcode.localization;

public interface Localizer {
    Pose2d getPoseEstimate();
    void setPoseEstimate(Pose2d estimate);
    void update();
}
