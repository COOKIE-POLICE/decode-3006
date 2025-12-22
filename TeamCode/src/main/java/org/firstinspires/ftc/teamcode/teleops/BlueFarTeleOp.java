package org.firstinspires.ftc.teamcode.teleops;

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
import org.firstinspires.ftc.teamcode.commands.GoToPoseCommand;
import org.firstinspires.ftc.teamcode.commands.HoodDownCommand;
import org.firstinspires.ftc.teamcode.commands.HoodUpCommand;
import org.firstinspires.ftc.teamcode.commands.IndexLeftCommand;
import org.firstinspires.ftc.teamcode.commands.IndexRightCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.LaunchCommand;
import org.firstinspires.ftc.teamcode.commands.OuttakeHoldCommand;
import org.firstinspires.ftc.teamcode.commands.OuttakePushCommand;
import org.firstinspires.ftc.teamcode.commands.PedroPathingDriveCommand;
import org.firstinspires.ftc.teamcode.commands.StopIndexCommand;
import org.firstinspires.ftc.teamcode.commands.StopIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.StopLaunchCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.HoodLifterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakerSubsystem;

import java.util.function.DoubleSupplier;

@Configurable
@TeleOp(name = "Blue Far TeleOp", group = "TeleOp")
public class BlueFarTeleOp extends CommandOpMode {
    private static final double TRIGGER_DEADZONE = 0.05;
    private static final double JOYSTICK_DEADZONE = 0.1;
    private static final Pose STARTING_POSE = Preferences.Poses.BLUE_FAR_START_POSE;

    private DoubleSupplier forward;
    private DoubleSupplier strafe;
    private DoubleSupplier rotate;

    private IntakeSubsystem intakeSubsystem;
    private IndexerSubsystem indexerSubsystem;
    private LauncherSubsystem launcherSubsystem;
    private HoodLifterSubsystem hoodLifterSubsystem;
    private OuttakerSubsystem outtakerSubsystem;
    private LimelightSubsystem limelightSubsystem;

    private PedroPathingDriveCommand normalDriveCommand;
    private GamepadEx driverOp;
    private IMU imu;
    private Follower follower;

    private Trigger joystickMovementTrigger;
    private Trigger rightTriggerActive;
    private Trigger leftTriggerActive;
    private Trigger noTriggersActive;

    @Override
    public void initialize() {
        _initializeHardware();
        _initializeSubsystems();
        _initializeInputSuppliers();
        _initializeTriggers();
        _configureDefaultCommand();
        _configureBindings();

        limelightSubsystem.switchPipeline(Preferences.Limelight.PIPELINE_BLUE_GOAL);
    }

    @Override
    public void run() {
        super.run();
        follower.update();
    }

    private void _initializeHardware() {
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
    }

    private void _initializeSubsystems() {
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        indexerSubsystem = new IndexerSubsystem(hardwareMap);
        launcherSubsystem = new LauncherSubsystem(hardwareMap);
        hoodLifterSubsystem = new HoodLifterSubsystem(hardwareMap);
        outtakerSubsystem = new OuttakerSubsystem(hardwareMap);
        limelightSubsystem = new LimelightSubsystem(hardwareMap, "limelight");
    }

    private void _initializeInputSuppliers() {
        forward = () -> driverOp.getLeftY();
        strafe = () -> -driverOp.getLeftX();
        rotate = () -> -driverOp.getRightX();
    }

    private void _initializeTriggers() {
        joystickMovementTrigger = new Trigger(this::_isJoystickMoving);

        rightTriggerActive = new Trigger(() ->
                driverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > TRIGGER_DEADZONE
        );

        leftTriggerActive = new Trigger(() ->
                driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > TRIGGER_DEADZONE
        );

        noTriggersActive = rightTriggerActive.negate().and(leftTriggerActive.negate());
    }

    private void _configureDefaultCommand() {
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
    }

    private void _configureBindings() {
        _configureIntakeBindings();
        _configureLauncherBindings();
        _configureIndexerBindings();
        _configureOuttakerBindings();
        _configureNavigationBindings();
    }

    private void _configureIntakeBindings() {
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
    }

    private void _configureLauncherBindings() {
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
    }

    private void _configureIndexerBindings() {
        rightTriggerActive.whileActiveContinuous(new IndexRightCommand(indexerSubsystem));
        leftTriggerActive.whileActiveContinuous(new IndexLeftCommand(indexerSubsystem));
        noTriggersActive.whileActiveContinuous(new StopIndexCommand(indexerSubsystem));
    }

    private void _configureOuttakerBindings() {
        driverOp.getGamepadButton(GamepadKeys.Button.Y)
                .toggleWhenPressed(
                        new OuttakePushCommand(outtakerSubsystem),
                        new OuttakeHoldCommand(outtakerSubsystem)
                );
    }

    private void _configureNavigationBindings() {
        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new GoToPoseCommand(follower, Preferences.Poses.BLUE_LAUNCH_POSE));

        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new GoToPoseCommand(follower, Preferences.Poses.BLUE_GATE_RELEASE_POSE));

        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new GoToPoseCommand(follower, Preferences.Poses.BLUE_PARK_POSE));
    }

    private boolean _isJoystickMoving() {
        return Math.abs(forward.getAsDouble()) > JOYSTICK_DEADZONE ||
                Math.abs(strafe.getAsDouble()) > JOYSTICK_DEADZONE ||
                Math.abs(rotate.getAsDouble()) > JOYSTICK_DEADZONE;
    }
}