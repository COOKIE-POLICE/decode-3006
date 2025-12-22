package org.firstinspires.ftc.teamcode.tests.manualintegrationtests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

@TeleOp(name = "Test Continuous Servo", group = "Manual Integration Tests")
public class TestContinuousServo extends OpMode {

    public static final double POWER_SENSITIVITY = 0.005;

    private final List<String> servoNames = new ArrayList<>();
    private final List<CRServo> servos = new ArrayList<>();

    private int currentServoIndex = 0;
    private double currentPower = 0.0;

    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastA = false;

    private CRServo activeServo;

    @Override
    public void init() {
        for (Map.Entry<String, CRServo> entry : hardwareMap.crservo.entrySet()) {
            String name = entry.getKey();
            servoNames.add(name);
            servos.add(entry.getValue());
        }
    }

    @Override
    public void loop() {
        if (servos.isEmpty()) {
            telemetry.addLine("No continuous servos detected");
            telemetry.update();
            return;
        }
        currentPower = Math.max(-1, Math.min(1, currentPower));
        if (gamepad1.a && !lastA) {
            reset();
        }
        lastA = gamepad1.a;

        handleServoSelection();
        activeServo = servos.get(currentServoIndex);
        handleServoControl(activeServo);
        displayTelemetry(activeServo);
    }

    private void handleServoSelection() {
        if (gamepad1.dpad_left && !lastDpadLeft) {
            currentServoIndex = (currentServoIndex - 1 + servos.size()) % servos.size();
        }
        if (gamepad1.dpad_right && !lastDpadRight) {
            currentServoIndex = (currentServoIndex + 1) % servos.size();
        }
        lastDpadLeft = gamepad1.dpad_left;
        lastDpadRight = gamepad1.dpad_right;
    }

    private void reset() {
        currentPower = 0;
    }

    private void handleServoControl(CRServo servo) {
        double delta = gamepad1.left_stick_y * -POWER_SENSITIVITY;
        currentPower += delta;
        servo.setPower(currentPower);
    }

    private void displayTelemetry(CRServo servo) {
        telemetry.addData("Active Servo", "%s (%d/%d)",
            servoNames.get(currentServoIndex),
            currentServoIndex + 1,
            servos.size());
        telemetry.addData("Current Power", "%.3f", currentPower);
        telemetry.addData("Power Range", "-1.000 to 1.000");
        telemetry.addLine();
        telemetry.addLine("=== Controls ===");
        telemetry.addLine("DPAD L/R: Change servo");
        telemetry.addLine("Left Stick Y: Adjust power");
        telemetry.addLine("A: Reset to 0 power");
        telemetry.update();
    }
}