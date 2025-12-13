package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class TestBenchColor {
    NormalizedColorSensor colorSensor;

    public enum detectedColor {
        RED,
        BLUE,
        YELLOW,
        UNKOWN
    }

    public void init(HardwareMap hwMap) {
        colorSensor = hwMap.get(NormalizedColorSensor.class, "colorSensor");
    }

    public DetectedColor getDetectedColor() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        float normRed, normGreen,normBlue;
        normRed = colors.red/
    }
}
