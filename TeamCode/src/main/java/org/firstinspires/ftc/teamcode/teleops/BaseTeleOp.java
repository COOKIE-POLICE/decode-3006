package org.firstinspires.ftc.teamcode.teleops;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.panels.Panels;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.pedroCommand.TurnToCommand;

import org.firstinspires.ftc.teamcode.Preferences;
import org.firstinspires.ftc.teamcode.commands.AdmitCommand;
import org.firstinspires.ftc.teamcode.commands.BlockCommand;
import org.firstinspires.ftc.teamcode.commands.EjectCommand;
import org.firstinspires.ftc.teamcode.commands.GoToPoseCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.LaunchCommand;
import org.firstinspires.ftc.teamcode.commands.PedroPathingDriveCommand;
import org.firstinspires.ftc.teamcode.commands.StopIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.StopLaunchCommand;
import org.firstinspires.ftc.teamcode.commands.TurnIndicatorGreenCommand;
import org.firstinspires.ftc.teamcode.commands.TurnIndicatorOffCommand;
import org.firstinspires.ftc.teamcode.commands.TurnToPoseCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.BlockerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndicatorSubsystem;

import java.util.function.DoubleSupplier;

@Configurable
public abstract class BaseTeleOp extends CommandOpMode {
    private static final double JOYSTICK_DEADZONE = 0.1;

    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    private Limelight3A limelight;

    private DoubleSupplier forward;
    private DoubleSupplier strafe;
    private DoubleSupplier rotate;

    private IntakeSubsystem intakeSubsystem;
    private LauncherSubsystem launcherSubsystem;
    private IndicatorSubsystem indicatorSubsystem;
    private BlockerSubsystem blockerSubsystem;

    private PedroPathingDriveCommand normalDriveCommand;
    private GamepadEx driverOp;
    private Follower follower;

    private Trigger joystickMovementTrigger;
    private Trigger blockingTrigger;

    protected abstract Pose getStartingPose();
    protected abstract Pose getGoalPose();
    protected abstract int getGoalPipeline();
    protected abstract Pose getLaunchPose();
    protected abstract Pose getCloseLaunchPose();
    protected abstract Pose getGateReleasePose();
    protected abstract Pose getParkPose();

    @Override
    public void initialize() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        limelight = hardwareMap.get(Limelight3A.class, Preferences.LIMELIGHT);
        limelight.start();

        limelight.pipelineSwitch(getGoalPipeline());
        follower = Constants.createFollower(hardwareMap);
        driverOp = new GamepadEx(gamepad1);

        follower.update();
        follower.startTeleopDrive();
        follower.setStartingPose(getStartingPose());

        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        launcherSubsystem = new LauncherSubsystem(hardwareMap, follower, getGoalPose());
        indicatorSubsystem = new IndicatorSubsystem(hardwareMap);
        blockerSubsystem = new BlockerSubsystem(hardwareMap);

        forward = () -> driverOp.getLeftY();
        strafe = () -> -driverOp.getLeftX();
        rotate = () -> -driverOp.getRightX();

        joystickMovementTrigger = new Trigger(this::isJoystickMoving);

        blockingTrigger = new Trigger(
                () -> blockerSubsystem.getBlocking()
        );
        normalDriveCommand = new PedroPathingDriveCommand(follower, forward, strafe, rotate);
        schedule(normalDriveCommand);
        schedule(new BlockCommand(blockerSubsystem));
        schedule(new TurnIndicatorOffCommand(indicatorSubsystem));

        configureButtonBindings();
    }


    private void configureButtonBindings() {
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

        blockingTrigger
                .whileActiveContinuous(new TurnIndicatorGreenCommand(indicatorSubsystem));

        blockingTrigger
                .negate()
                .whileActiveContinuous(new TurnIndicatorOffCommand(indicatorSubsystem));

        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new GoToPoseCommand(follower, getLaunchPose()));

        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new GoToPoseCommand(follower, getGateReleasePose()));

        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(
                        new GoToPoseCommand(follower, getLaunchPose())
                );

        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(
                        new GoToPoseCommand(follower, getCloseLaunchPose())
                );

        driverOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .toggleWhenPressed(
                        new BlockCommand(blockerSubsystem),
                        new AdmitCommand(blockerSubsystem)
                );

        driverOp.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new TurnToPoseCommand(follower, getGoalPose()));
    }

    @Override
    public void run() {
        telemetryM.addData("Current Velocity: ", launcherSubsystem.getVelocityMagnitude());
        telemetryM.addData("Target Velocity: ", launcherSubsystem.getTargetVelocity());
        telemetryM.update(telemetry);
        super.run();
        follower.update();
    }
    @Override
    public void end() {
        schedule(new BlockCommand(blockerSubsystem));
    }

    private boolean isJoystickMoving() {
        return Math.abs(forward.getAsDouble()) > JOYSTICK_DEADZONE ||
                Math.abs(strafe.getAsDouble()) > JOYSTICK_DEADZONE ||
                Math.abs(rotate.getAsDouble()) > JOYSTICK_DEADZONE;
    }
}