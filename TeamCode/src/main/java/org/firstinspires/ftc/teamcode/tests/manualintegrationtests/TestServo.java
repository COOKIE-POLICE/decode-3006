package org.firstinspires.ftc.teamcode.tests.manualintegrationtests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

// Test servo teleop cuz don't got servo tester
@TeleOp(name = "Test Servo", group = "Manual Integration Tests")
public class TestServo extends OpMode {

    private final List<String> servoNames = new ArrayList<>();
    private final List<Servo> servos = new ArrayList<>();
    private final double joystickSensitivity = 0.005;
    private int currentIndex = 0;

    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastA = false, lastB = false, lastX = false, lastY = false;

    private double currentPosition = 0.5;

    @Override
    public void init() {
        for (Map.Entry<String, Servo> entry : hardwareMap.servo.entrySet()) {
            String name = entry.getKey();
            servoNames.add(name);
            servos.add(entry.getValue());
        }
    }

    @Override
    public void loop() {
        if (servos.isEmpty()) {
            telemetry.addLine("No servos detected");
            telemetry.update();
            return;
        }

        if (gamepad1.dpad_left && !lastDpadLeft) {
            currentIndex = (currentIndex - 1 + servos.size()) % servos.size();
        }
        if (gamepad1.dpad_right && !lastDpadRight) {
            currentIndex = (currentIndex + 1) % servos.size();
        }
        lastDpadLeft = gamepad1.dpad_left;
        lastDpadRight = gamepad1.dpad_right;

        if (gamepad1.a && !lastA) currentPosition = 0.0;
        if (gamepad1.b && !lastB) currentPosition = 0.5;
        if (gamepad1.y && !lastY) currentPosition = 1.0;
        if (gamepad1.x && !lastX) currentPosition = 0.25;

        lastA = gamepad1.a;
        lastB = gamepad1.b;
        lastX = gamepad1.x;
        lastY = gamepad1.y;

        double delta = gamepad1.left_stick_x * joystickSensitivity;
        currentPosition += delta;
        currentPosition = Math.max(0.0, Math.min(1.0, currentPosition));

        Servo active = servos.get(currentIndex);
        active.setPosition(currentPosition);

        telemetry.addData("Active Servo", "%s (%d/%d)",
            servoNames.get(currentIndex),
            currentIndex + 1,
            servos.size());
        telemetry.addData("Current Position", "%.3f", currentPosition);
        telemetry.addData("Range", "0.000 - 1.000");
        telemetry.addLine();
        telemetry.addLine("=== Controls ===");
        telemetry.addLine("DPAD L/R: Change servo");
        telemetry.addLine("Left Stick X: Adjust position");
        telemetry.addLine("A: 0.0 | B: 0.5");
        telemetry.addLine("X: 0.25 | Y: 1.0");
        telemetry.update();
    }
}
