package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "Flywheel Tuner", group = "Tuning")
public class FlywheelTuner extends OpMode {

    public static double proportionalGain = 0.0;
    public static double integralGain = 0.0;
    public static double derivativeGain = 0.0;
    public static double feedforwardGain = 0.0;
    public static double targetVelocity = 0.0;
    public static String motorHardwareName = "launcherMotor";

    private DcMotorEx launcherMotor;
    private ElapsedTime timer;
    private String currentMotorName = "";

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        timer = new ElapsedTime();

        // initializeMotor()
        try {
            launcherMotor = hardwareMap.get(DcMotorEx.class, motorHardwareName);
            launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            currentMotorName = motorHardwareName;
        } catch (IllegalArgumentException exception) {
            launcherMotor = null;
            currentMotorName = "";
        }
    }

    @Override
    public void init_loop() {
        if (!motorHardwareName.equals(currentMotorName)) {
            // initializeMotor()
            try {
                launcherMotor = hardwareMap.get(DcMotorEx.class, motorHardwareName);
                launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                currentMotorName = motorHardwareName;
            } catch (IllegalArgumentException exception) {
                launcherMotor = null;
                currentMotorName = "";
            }
        }

        telemetry.addData("Motor Status", launcherMotor != null ? "Connected" : "Not Found");
        telemetry.addData("Motor Name", motorHardwareName);
        telemetry.update();
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {
        if (!motorHardwareName.equals(currentMotorName)) {
            try {
                launcherMotor = hardwareMap.get(DcMotorEx.class, motorHardwareName);
                launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                currentMotorName = motorHardwareName;
            } catch (IllegalArgumentException exception) {
                launcherMotor = null;
                currentMotorName = "";
            }
        }

        if (launcherMotor == null) {
            telemetry.addData("Error", "Motor not found: " + motorHardwareName);
            telemetry.update();
            return;
        }

        launcherMotor.setVelocityPIDFCoefficients(
                proportionalGain,
                integralGain,
                derivativeGain,
                feedforwardGain
        );

        launcherMotor.setVelocity(targetVelocity);
        double currentVelocity = launcherMotor.getVelocity();
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Current Velocity", currentVelocity);
        telemetry.update();
    }
}
