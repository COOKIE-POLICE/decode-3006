package org.firstinspires.ftc.teamcode.modules.colorsensors;

import android.graphics.Color;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

public class HsvColorSensor {

    private final NormalizedColorSensor colorSensor;

    public HsvColorSensor(HardwareMap hardwareMap, String deviceName) {
        this.colorSensor = hardwareMap.get(
            NormalizedColorSensor.class,
            deviceName
        );
    }

    public float getHue() {
        float[] hsv = new float[3];
        Color.colorToHSV(colorSensor.getNormalizedColors().toColor(), hsv);
        return hsv[0];
    }

    public float getSaturation() {
        float[] hsv = new float[3];
        Color.colorToHSV(colorSensor.getNormalizedColors().toColor(), hsv);
        return hsv[1];
    }

    public float getValue() {
        float[] hsv = new float[3];
        Color.colorToHSV(colorSensor.getNormalizedColors().toColor(), hsv);
        return hsv[2];
    }
    public int getColor() {
        float[] hsv = new float[3];
        Color.colorToHSV(colorSensor.getNormalizedColors().toColor(), hsv);

        float hue = hsv[0];
        float saturation = hsv[1];
        float value = hsv[2];

        if (value < 0.01 || saturation < 0.1) {
            return Color.TRANSPARENT;
        }
        if (hue >= 200 && hue <= 260) {
            return Color.MAGENTA;
        }
        else if (hue >= 120 && hue <= 160) {
            return Color.GREEN;
        }
        return Color.TRANSPARENT;
    }

    public boolean matchColor(int targetColor) {
        float[] hsv = new float[3];
        Color.colorToHSV(colorSensor.getNormalizedColors().toColor(), hsv);

        float hue = hsv[0];
        float saturation = hsv[1];
        float value = hsv[2];

        if (value < 0.01 || saturation < 0.1) {
            return false;
        }

        if (targetColor == Color.MAGENTA) {
            return (hue >= 200 && hue <= 260);
        } else if (targetColor == Color.GREEN) {
            return (hue >= 120 && hue <= 160);
        } else {
            return false;
        }
    }
}
