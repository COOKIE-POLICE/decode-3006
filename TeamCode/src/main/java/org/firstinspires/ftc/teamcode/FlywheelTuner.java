package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.modules.TunedMotor;

// 192.168.43.1:8080/dash
@Config
@TeleOp(name = "Flywheel Tuner", group = "Tuning")
public class FlywheelTuner extends OpMode {

    public static double proportionalGain = 0.0;
    public static double integralGain = 0.0;
    public static double derivativeGain = 0.0;
    public static double feedforwardGain = 0.0;
    public static double targetVelocity = 0.0;
    public static String motorHardwareName = "launcherMotor";

    private TunedMotor tunedMotor;
    private String currentMotorName = "";

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        _initializeMotor();
    }

    @Override
    public void init_loop() {
        if (!motorHardwareName.equals(currentMotorName)) {
            _initializeMotor();
        }

        telemetry.addData("Motor Status", tunedMotor != null ? "Connected" : "Not Found");
        telemetry.addData("Motor Name", motorHardwareName);
        telemetry.update();
    }

    @Override
    public void loop() {
        if (!motorHardwareName.equals(currentMotorName)) {
            _initializeMotor();
        }

        if (tunedMotor == null) {
            telemetry.addData("Error", "Motor not found: " + motorHardwareName);
            telemetry.update();
            return;
        }
        tunedMotor.setPIDF(proportionalGain, integralGain, derivativeGain, feedforwardGain);
        tunedMotor.setTargetVelocity(targetVelocity);

        double currentVelocity = Math.abs(tunedMotor.getMotor().getVelocity());
        tunedMotor.update(currentVelocity);

        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Current Velocity", currentVelocity);
        telemetry.addData("Error", Math.abs(targetVelocity - currentVelocity));
        telemetry.addData("At Setpoint", tunedMotor.atSetPoint());
        telemetry.update();
    }

    private void _initializeMotor() {
        try {
            DcMotorEx motor = hardwareMap.get(DcMotorEx.class, motorHardwareName);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            tunedMotor = new TunedMotor(motor, proportionalGain, integralGain, derivativeGain, feedforwardGain);
            currentMotorName = motorHardwareName;
        } catch (IllegalArgumentException exception) {
            tunedMotor = null;
            currentMotorName = "";
        }
    }
}