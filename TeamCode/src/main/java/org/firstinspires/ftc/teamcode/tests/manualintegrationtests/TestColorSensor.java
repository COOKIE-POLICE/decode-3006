package org.firstinspires.ftc.teamcode.tests.manualintegrationtests;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.modules.colorsensors.HsvColorSensor;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

// Test servo teleop cuz don't got servo tester
@TeleOp(name = "Test Color Sensor", group = "Manual Integration Tests")
public class TestColorSensor extends OpMode {

    private final List<String> colorSensorNames = new ArrayList<>();
    private final List<HsvColorSensor> colorSensors = new ArrayList<>();
    private int currentIndex = 0;

    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;

    @Override
    public void init() {
        for (Map.Entry<String, ColorSensor> entry : hardwareMap.colorSensor.entrySet()) {
            String name = entry.getKey();
            colorSensorNames.add(name);
            colorSensors.add(new HsvColorSensor(hardwareMap, name));
        }
    }

    @Override
    public void loop() {
        if (colorSensors.isEmpty()) {
            telemetry.addLine("No color sensors detected");
            telemetry.update();
            return;
        }
        if (gamepad1.dpad_left && !lastDpadLeft) {
            currentIndex = (currentIndex - 1 + colorSensors.size()) % colorSensors.size();
        }
        if (gamepad1.dpad_right && !lastDpadRight) {
            currentIndex = (currentIndex + 1) % colorSensors.size();
        }
        lastDpadLeft = gamepad1.dpad_left;
        lastDpadRight = gamepad1.dpad_right;
        HsvColorSensor active = colorSensors.get(currentIndex);

        telemetry.addData("Active Sensor", "%s (%d/%d)",
            colorSensorNames.get(currentIndex),
            currentIndex + 1,
            colorSensors.size());
        telemetry.addLine();
        telemetry.addData("Hue", "%.1f", active.getHue());
        telemetry.addData("Saturation", "%.3f", active.getSaturation());
        telemetry.addData("Value", "%.3f", active.getValue());
        telemetry.addLine();
        telemetry.addData("Matches Green", active.matchColor(Color.GREEN));
        telemetry.addData("Matches Magenta", active.matchColor(Color.MAGENTA));
        telemetry.addLine();
        telemetry.addLine("=== Controls ===");
        telemetry.addLine("DPAD L/R: Change sensor");
        telemetry.update();
    }
}
