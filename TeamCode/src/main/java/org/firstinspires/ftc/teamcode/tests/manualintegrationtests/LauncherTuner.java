package org.firstinspires.ftc.teamcode.tests.manualintegrationtests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Preferences;
@TeleOp(name = "Launcher Tuner", group = "Manual Integration Tests")
public class LauncherTuner extends OpMode {

    private DcMotorEx motor;

    private double targetVelocity = Preferences.Launcher.LAUNCH_VELOCITY;

    private double p = 0.0025;
    private double d = 0.00015;
    private double f = 13;

    private final double[] pidStepSizes = {0.00001, 0.0001, 0.001, 0.01};
    private final double[] velocityStepSizes = {1, 5, 10, 25, 50};

    private int pidStepIndex = 2;
    private int velocityStepIndex = 2;

    private boolean prevB = false;
    private boolean prevX = false;

    private double lastP = Double.NaN;
    private double lastD = Double.NaN;
    private double lastF = Double.NaN;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, Preferences.LAUNCHER_MOTOR);
        motor.setDirection(Preferences.Launcher.DIRECTION);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Preferences.Launcher.PIDF_COEFFICIENTS);
    }

    @Override
    public void loop() {
        handleStepSizeChange();
        handlePIDFAdjustment();
        handleVelocityAdjustment();
        updateMotor();
        displayTelemetry();
    }

    private void handleStepSizeChange() {
        boolean b = gamepad1.b;
        if (b && !prevB) {
            pidStepIndex = (pidStepIndex + 1) % pidStepSizes.length;
        }
        prevB = b;

        boolean x = gamepad1.x;
        if (x && !prevX) {
            velocityStepIndex = (velocityStepIndex + 1) % velocityStepSizes.length;
        }
        prevX = x;
    }

    private void handlePIDFAdjustment() {
        double step = pidStepSizes[pidStepIndex];

        if (gamepad1.dpad_up) {
            p += step;
        }
        if (gamepad1.dpad_down) {
            p -= step;
        }
        if (gamepad1.dpad_right) {
            f += step;
        }
        if (gamepad1.dpad_left) {
            f -= step;
        }
    }

    private void handleVelocityAdjustment() {
        double step = velocityStepSizes[velocityStepIndex];

        if (gamepad1.left_bumper) {
            targetVelocity -= step;
        }
        if (gamepad1.right_bumper) {
            targetVelocity += step;
        }

        if (gamepad1.y) {
            targetVelocity = 0;
            p = 0;
            d = 0;
            f = 0;
        }
    }

    private void updateMotor() {
        if (p != lastP || d != lastD || f != lastF) {
            motor.setPIDFCoefficients(
                    DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(p, 0, d, f)
            );
            lastP = p;
            lastD = d;
            lastF = f;
        }
        motor.setVelocity(targetVelocity);
    }

    private void displayTelemetry() {
        double currentVelocity = Math.abs(motor.getVelocity());
        double error = targetVelocity != 0
                ? Math.abs(currentVelocity - Math.abs(targetVelocity)) / Math.abs(targetVelocity) * 100
                : 0;

        telemetry.addLine("=== Controls ===");
        telemetry.addLine("D-Pad Up/Down: Adjust P");
        telemetry.addLine("D-Pad Left/Right: Adjust F");
        telemetry.addLine("LB/RB: Adjust Velocity");
        telemetry.addLine("B: Cycle PID Step");
        telemetry.addLine("X: Cycle Velocity Step");
        telemetry.addLine("Y: Reset");
        telemetry.addLine();

        telemetry.addLine("=== PIDF ===");
        telemetry.addData("P", "%.6f", p);
        telemetry.addData("D", "%.6f", d);
        telemetry.addData("F", "%.6f", f);
        telemetry.addData("PID Step", pidStepSizes[pidStepIndex]);

        telemetry.addLine();
        telemetry.addLine("=== Velocity ===");
        telemetry.addData("Target", "%.2f", Math.abs(targetVelocity));
        telemetry.addData("Current", "%.2f", currentVelocity);
        telemetry.addData("Error", "%.2f%%", error);
        telemetry.addData("Velocity Step", velocityStepSizes[velocityStepIndex]);

        telemetry.update();
    }
}
