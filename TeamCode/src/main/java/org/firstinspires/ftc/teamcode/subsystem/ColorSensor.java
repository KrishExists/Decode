package org.firstinspires.ftc.teamcode.subsystem;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorSensor implements Subsystem{

    private final com.qualcomm.robotcore.hardware.ColorSensor colorSensor;
    private final DistanceSensor distanceSensor;

    private final float[] hsv = new float[3];

    public ColorSensor(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(com.qualcomm.robotcore.hardware.ColorSensor.class, "colorSensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "colorSensor");
    }

    /* =====================
       RAW SENSOR DATA
       ===================== */

    public int getRed() {
        return colorSensor.red();
    }

    public int getGreen() {
        return colorSensor.green();
    }

    public int getBlue() {
        return colorSensor.blue();
    }

    public int[] getRGB() {
        return new int[]{getRed(), getGreen(), getBlue()};
    }

    /* =====================
       HSV CONVERSION
       ===================== */

    public float[] getHSV() {
        Color.RGBToHSV(getRed(), getGreen(), getBlue(), hsv);
        return hsv;
    }

    public float getHue() {
        return getHSV()[0];
    }

    public float getSaturation() {
        return getHSV()[1];
    }

    public float getValue() {
        return getHSV()[2];
    }

    /* =====================
       COLOR DETECTION
       ===================== */

    public String getDetectedColor() {
        float hue = getHue();
        float saturation = getSaturation();
        float value = getValue();

        if (value < 0.2) {
            return "BLACK";
        } else if (saturation < 0.2) {
            return "WHITE";
        } else if (hue < 30 || hue > 330) {
            return "RED";
        } else if (hue < 90) {
            return "YELLOW";
        } else if (hue < 150) {
            return "GREEN";
        } else if (hue < 270) {
            return "BLUE";
        }

        return "UNKNOWN";
    }

    /* =====================
       DISTANCE
       ===================== */

    public double getDistanceCM() {
        return distanceSensor.getDistance(DistanceUnit.CM);
    }

    @Override
    public void init() {

    }

    @Override
    public void update(Gamepad gamepad1, Gamepad gamepad2) {

    }
}
