package org.firstinspires.ftc.teamcode.Projects;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class coloorsensHWmap {

    public ColorSensor sensor1;
    public ColorSensor sensor2;

    public void init(HardwareMap hwMap) {
        // "sensor1" must exactly match the name in your Driver Hub configuration
        sensor1 = hwMap.get(ColorSensor.class, "sensor1");
        sensor2 = hwMap.get(ColorSensor.class, "sensor2");

    }
}

