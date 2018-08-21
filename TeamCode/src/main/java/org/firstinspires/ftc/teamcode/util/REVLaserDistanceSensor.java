package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import org.firstinspires.ftc.teamcode.util.VL53L0X;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;

/**
 * {@link REVLaserDistanceSensor} implements support for the REV Robotics time-of-flight distance sensor.
 *
 * @see <a href="http://revrobotics.com">REV Robotics Website</a>
 *
 */
@SuppressWarnings({"WeakerAccess", "unused"}) // Ignore access and unused warnings
@I2cSensor(name = "REV LASER Range Sensor", description = "REV Time of Flight Distance Sensor ", xmlTag = "REV_VL53L0X_RANGE_SENSOR")
public class REVLaserDistanceSensor extends VL53L0X {
    public REVLaserDistanceSensor(I2cDeviceSynch deviceClient) {
        super(deviceClient);
    }

    @Override
    public String getDeviceName() {
        return "REV_Time-of-Flight_Distance_Sensor";
    }
}
