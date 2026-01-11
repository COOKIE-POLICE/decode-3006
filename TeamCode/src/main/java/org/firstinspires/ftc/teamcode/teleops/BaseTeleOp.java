package org.firstinspires.ftc.teamcode.teleops;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Preferences;
import org.firstinspires.ftc.teamcode.commands.EjectCommand;
import org.firstinspires.ftc.teamcode.commands.GoToPoseCommand;
import org.firstinspires.ftc.teamcode.commands.HoodDownCommand;
import org.firstinspires.ftc.teamcode.commands.HoodUpCommand;
import org.firstinspires.ftc.teamcode.commands.IndexLeftCommand;
import org.firstinspires.ftc.teamcode.commands.IndexRightCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.LaunchCommand;
import org.firstinspires.ftc.teamcode.commands.PedroPathingDriveCommand;
import org.firstinspires.ftc.teamcode.commands.StopIndexCommand;
import org.firstinspires.ftc.teamcode.commands.StopIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.StopLaunchCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.HoodLifterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakerSubsystem;

import java.util.function.DoubleSupplier;

@Configurable
public abstract class BaseTeleOp extends CommandOpMode {
    private static final double TRIGGER_DEADZONE = 0.05;
    private static final double JOYSTICK_DEADZONE = 0.1;

    private DoubleSupplier forward;
    private DoubleSupplier strafe;
    private DoubleSupplier rotate;

    private IntakeSubsystem intakeSubsystem;
    private IndexerSubsystem indexerSubsystem;
    private LauncherSubsystem launcherSubsystem;
    private HoodLifterSubsystem hoodLifterSubsystem;
    private OuttakerSubsystem outtakerSubsystem;

    private PedroPathingDriveCommand normalDriveCommand;
    private GamepadEx driverOp;
    private IMU imu;
    private Follower follower;

    private Trigger joystickMovementTrigger;
    private Trigger rightTriggerActive;
    private Trigger leftTriggerActive;
    private Trigger noTriggersActive;
    private Trigger atLaunchVelocityTrigger;

    protected abstract Pose getStartingPose();
    protected abstract Pose getGoalPose();
    protected abstract Pose getLaunchPose();
    protected abstract Pose getCloseLaunchPose();
    protected abstract Pose getGateReleasePose();
    protected abstract Pose getParkPose();

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
        follower.setStartingPose(getStartingPose());

        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        indexerSubsystem = new IndexerSubsystem(hardwareMap);
        launcherSubsystem = new LauncherSubsystem(hardwareMap, follower, getGoalPose());
        hoodLifterSubsystem = new HoodLifterSubsystem(hardwareMap);
        outtakerSubsystem = new OuttakerSubsystem(hardwareMap);

        forward = () -> driverOp.getLeftY();
        strafe = () -> -driverOp.getLeftX();
        rotate = () -> -driverOp.getRightX();

        joystickMovementTrigger = new Trigger(this::_isJoystickMoving);

        rightTriggerActive = new Trigger(() ->
                driverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > TRIGGER_DEADZONE
        );

        leftTriggerActive = new Trigger(() ->
                driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > TRIGGER_DEADZONE
        );

        atLaunchVelocityTrigger = new Trigger(
                () -> launcherSubsystem.isAtTargetVelocity()
        );

        noTriggersActive = rightTriggerActive.negate().and(leftTriggerActive.negate());

        normalDriveCommand = new PedroPathingDriveCommand(follower, forward, strafe, rotate);
        schedule(normalDriveCommand);

        _configureButtonBindings();
    }

    private void _configureButtonBindings() {
        joystickMovementTrigger.whileActiveContinuous(
                new InstantCommand(() -> {
                    if (!follower.isTeleopDrive()) {
                        follower.breakFollowing();
                        follower.startTeleopDrive();
                    }
                })
        );

        driverOp.getGamepadButton(GamepadKeys.Button.A)
                .toggleWhenPressed(
                        new IntakeCommand(intakeSubsystem),
                        new StopIntakeCommand(intakeSubsystem)
                );

        driverOp.getGamepadButton(GamepadKeys.Button.X)
                .toggleWhenPressed(
                        new EjectCommand(intakeSubsystem),
                        new StopIntakeCommand(intakeSubsystem)
                );

        driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .toggleWhenPressed(
                        new LaunchCommand(launcherSubsystem),
                        new StopLaunchCommand(launcherSubsystem)
                );

        driverOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .toggleWhenPressed(
                        new HoodUpCommand(hoodLifterSubsystem),
                        new HoodDownCommand(hoodLifterSubsystem)
                );

        rightTriggerActive.whenActive(new IndexRightCommand(indexerSubsystem));
        leftTriggerActive.whenActive(new IndexLeftCommand(indexerSubsystem));
        noTriggersActive.whenActive(new StopIndexCommand(indexerSubsystem));

        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new GoToPoseCommand(follower, getLaunchPose()));

        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new GoToPoseCommand(follower, getGateReleasePose()));

        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new GoToPoseCommand(follower, getParkPose()));

        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new GoToPoseCommand(follower, getCloseLaunchPose()));
    }

    @Override
    public void run() {
        telemetry.addLine("=== LAUNCHER STATUS ===");
        telemetry.addData("Is Launching", launcherSubsystem.isLaunching());
        telemetry.addData("At Target", launcherSubsystem.isAtTargetVelocity());
        telemetry.addData("Launcher F", launcherSubsystem.getF());
        telemetry.addLine();

        telemetry.addLine("=== VELOCITY ===");
        telemetry.addData("Current Velocity", "%.2f", launcherSubsystem.getVelocityMagnitude());
        telemetry.addData("Target Velocity", "%.2f", launcherSubsystem.getTargetVelocity());
        telemetry.addData("Velocity Error", "%.2f", launcherSubsystem.getVelocityError());
        telemetry.addData("Percent Error", "%.2f%%", launcherSubsystem.getPercentError());
        telemetry.addLine();

        telemetry.addLine("=== POSITION ===");
        telemetry.addData("Distance to Goal", "%.2f", launcherSubsystem.getDistanceToGoal());
        telemetry.addData("Current Pose", launcherSubsystem.getCurrentPose().toString());
        telemetry.addLine();

        telemetry.addLine("=== POWER ===");
        telemetry.addData("Motor Power", "%.3f", launcherSubsystem.getPower());
        telemetry.addData("Battery Voltage", "%.2fV", launcherSubsystem.getBatteryVoltage());

        telemetry.update();
        super.run();
        follower.update();
    }

    private boolean _isJoystickMoving() {
        return Math.abs(forward.getAsDouble()) > JOYSTICK_DEADZONE ||
                Math.abs(strafe.getAsDouble()) > JOYSTICK_DEADZONE ||
                Math.abs(rotate.getAsDouble()) > JOYSTICK_DEADZONE;
    }
}