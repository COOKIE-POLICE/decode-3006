package org.firstinspires.ftc.teamcode.tests.manualintegrationtests.launcher;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Preferences;
import org.firstinspires.ftc.teamcode.commands.EjectCommand;
import org.firstinspires.ftc.teamcode.commands.HoodDownCommand;
import org.firstinspires.ftc.teamcode.commands.HoodUpCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.PedroPathingDriveCommand;
import org.firstinspires.ftc.teamcode.commands.StopIntakeCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.HoodLifterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;

import java.util.function.DoubleSupplier;

@Configurable
@TeleOp(name = "Launch Data TeleOp", group = "Manual Integration Tests")
public class LaunchDataTeleOp extends CommandOpMode {
    private static final double JOYSTICK_DEADZONE = 0.1;
    private static final Pose STARTING_POSE = Preferences.Poses.BLUE_FAR_START_POSE;
    private static final double POWER_INCREMENT_LARGE = 0.1;
    private static final double POWER_INCREMENT_MEDIUM = 0.01;
    private static final double MIN_POWER = 0.0;
    private static final double MAX_POWER = 1.0;

    private DoubleSupplier forward;
    private DoubleSupplier strafe;
    private DoubleSupplier rotate;

    private LauncherSubsystem launcherSubsystem;
    private HoodLifterSubsystem hoodLifterSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private PedroPathingDriveCommand normalDriveCommand;
    private GamepadEx driverOp;
    private IMU imu;
    private Follower follower;
    private Trigger joystickMovementTrigger;

    private double targetPower;

    @Override
    public void initialize() {
        follower = Constants.createFollower(hardwareMap);
        driverOp = new GamepadEx(gamepad1);

        imu = hardwareMap.get(IMU.class, Preferences.IMU);
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        ));

        follower.update();
        follower.startTeleopDrive();
        follower.setStartingPose(STARTING_POSE);

        launcherSubsystem = new LauncherSubsystem(hardwareMap, follower, Preferences.Poses.BLUE_GOAL_POSE);
        hoodLifterSubsystem = new HoodLifterSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);

        forward = () -> driverOp.getLeftY();
        strafe = () -> -driverOp.getLeftX();
        rotate = () -> -driverOp.getRightX();

        targetPower = 0.0;

        joystickMovementTrigger = new Trigger(this::_isJoystickMoving);

        normalDriveCommand = new PedroPathingDriveCommand(follower, forward, strafe, rotate);
        schedule(normalDriveCommand);

        joystickMovementTrigger.whileActiveContinuous(
                new InstantCommand(() -> {
                    if (!follower.isTeleopDrive()) {
                        follower.breakFollowing();
                        follower.startTeleopDrive();
                    }
                })
        );

        driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .toggleWhenPressed(
                        new InstantCommand(this::_startLaunching),
                        new InstantCommand(this::_stopLaunching)
                );

        driverOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .toggleWhenPressed(
                        new HoodUpCommand(hoodLifterSubsystem),
                        new HoodDownCommand(hoodLifterSubsystem)
                );

        driverOp.getGamepadButton(GamepadKeys.Button.A)
                .toggleWhenPressed(
                        new IntakeCommand(intakeSubsystem),
                        new StopIntakeCommand(intakeSubsystem)
                );

        driverOp.getGamepadButton(GamepadKeys.Button.B)
                .toggleWhenPressed(
                        new EjectCommand(intakeSubsystem),
                        new StopIntakeCommand(intakeSubsystem)
                );

        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> _adjustPower(POWER_INCREMENT_LARGE)));

        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(() -> _adjustPower(POWER_INCREMENT_MEDIUM)));

        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(this::_resetPower));
    }

    @Override
    public void run() {
        telemetry.addLine("=== LAUNCHER DATA ===");
        telemetry.addData("Distance to Goal", "%.2f", launcherSubsystem.getDistanceToGoal());
        telemetry.addData("Current Velocity", "%.2f", launcherSubsystem.getVelocityMagnitude());
        telemetry.addData("Current Power", "%.2f", launcherSubsystem.getPower());
        telemetry.addData("Target Velocity", "%.2f", launcherSubsystem.getTargetVelocity());
        telemetry.addLine();

        telemetry.addLine("=== PIDF COEFFICIENTS ===");
        telemetry.addData("P", "%.6f", launcherSubsystem.getP());
        telemetry.addData("I", "%.6f", launcherSubsystem.getI());
        telemetry.addData("D", "%.6f", launcherSubsystem.getD());
        telemetry.addData("F (Feedforward)", "%.6f", launcherSubsystem.getF());
        telemetry.addData("Battery Voltage", "%.2fV", launcherSubsystem.getBatteryVoltage());
        telemetry.addLine();

        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("Right Bumper: Toggle Launch | Left Bumper: Toggle Hood");
        telemetry.addLine("A: Toggle Intake | B: Toggle Eject");
        telemetry.addLine("DPAD UP: +0.1 | DPAD RIGHT: +0.01 | DPAD DOWN: Reset");

        telemetry.update();
        super.run();
        follower.update();
    }

    private void _adjustPower(double delta) {
        targetPower = Math.max(MIN_POWER, Math.min(MAX_POWER, targetPower + delta));
        launcherSubsystem.setPower(targetPower);
    }

    private void _resetPower() {
        targetPower = 0.0;
        launcherSubsystem.setPower(targetPower);
    }

    private void _startLaunching() {
        launcherSubsystem.setPower(targetPower);
    }

    private void _stopLaunching() {
        launcherSubsystem.setPower(0.0);
    }

    private boolean _isJoystickMoving() {
        return Math.abs(forward.getAsDouble()) > JOYSTICK_DEADZONE ||
                Math.abs(strafe.getAsDouble()) > JOYSTICK_DEADZONE ||
                Math.abs(rotate.getAsDouble()) > JOYSTICK_DEADZONE;
    }
}