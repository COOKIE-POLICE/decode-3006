package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.BlockerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndicatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemFactory;

import java.util.function.DoubleSupplier;

@Config
public abstract class BaseTeleOp extends CommandOpMode {

    private static final double JOYSTICK_DEADZONE = 0.1;

    public static double lockOnProportional = 0.005;
    public static double lockOnIntegral = 0.0;
    public static double lockOnDerivative = 0.0;
    public static double lockOnFeedforward = 0.0;
    public static PIDFController lockOnPidf = new PIDFController(lockOnProportional, lockOnIntegral, lockOnDerivative, lockOnFeedforward);

    private IntakeSubsystem intakeSubsystem;
    private LauncherSubsystem launcherSubsystem;
    private IndicatorSubsystem indicatorSubsystem;
    private BlockerSubsystem blockerSubsystem;
    private LimelightSubsystem limelightSubsystem;
    private PedroPathingDriveCommand normalDriveCommand;
    private GamepadEx driverOp;
    private Follower follower;
    private DoubleSupplier forward;
    private DoubleSupplier strafe;
    private DoubleSupplier rotate;
    private Trigger joystickMovementTrigger;
    private Trigger blockingTrigger;
    private boolean isLockedOn = false;

    protected abstract Pose getStartingPose();
    protected abstract Pose getGoalPose();
    protected abstract int getGoalPipeline();
    protected abstract Pose getLaunchPose();
    protected abstract Pose getCloseLaunchPose();

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = Constants.createFollower(hardwareMap);
        follower.update();
        follower.startTeleopDrive();
        follower.setStartingPose(getStartingPose());

        driverOp = new GamepadEx(gamepad1);

        intakeSubsystem = SubsystemFactory.createIntake(hardwareMap);
        launcherSubsystem = SubsystemFactory.createLauncher(hardwareMap, follower, getGoalPose());
        indicatorSubsystem = SubsystemFactory.createIndicator(hardwareMap);
        blockerSubsystem = SubsystemFactory.createBlocker(hardwareMap);
        limelightSubsystem = SubsystemFactory.createLimelight(hardwareMap);

        lockOnPidf.setTolerance(0.0);
        limelightSubsystem.switchPipeline(getGoalPipeline());

        forward = () -> driverOp.getLeftY();
        strafe = () -> -driverOp.getLeftX();
        rotate = this::getRotation;

        joystickMovementTrigger = new Trigger(this::isJoystickMoving);
        blockingTrigger = new Trigger(() -> blockerSubsystem.isBlocking());

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
                .whileActiveContinuous(new TurnIndicatorOffCommand(indicatorSubsystem));

        blockingTrigger
                .negate()
                .whileActiveContinuous(new TurnIndicatorGreenCommand(indicatorSubsystem));

        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new GoToPoseCommand(follower, getLaunchPose()));

        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new GoToPoseCommand(follower, getCloseLaunchPose()));

        driverOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenHeld(new AdmitCommand(blockerSubsystem))
                .whenReleased(new BlockCommand(blockerSubsystem));

        driverOp.getGamepadButton(GamepadKeys.Button.Y)
                .toggleWhenPressed(
                        new RunCommand(() -> isLockedOn = true),
                        new RunCommand(() -> isLockedOn = false)
                );

        new Trigger(() -> driverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0)
                .whenActive(new IntakeCommand(intakeSubsystem))
                .whenInactive(new StopIntakeCommand(intakeSubsystem));
    }

    @Override
    public void run() {
        super.run();
        telemetry.addData("Limelight TX", limelightSubsystem.getTargetX());
        telemetry.addData("Zero Line", 0.0);
        telemetry.update();
        follower.update();
    }

    @Override
    public void end() {
        schedule(new BlockCommand(blockerSubsystem));
        limelightSubsystem.shutdown();
    }

    private double getRotation() {
        if (isLockedOn && limelightSubsystem.hasValidTarget()) {
            lockOnPidf.setPIDF(lockOnProportional, lockOnIntegral, lockOnDerivative, lockOnFeedforward);
            return lockOnPidf.calculate(limelightSubsystem.getTargetX(), 0.0);
        }
        return -driverOp.getRightX();
    }

    private boolean isJoystickMoving() {
        return Math.abs(forward.getAsDouble()) > JOYSTICK_DEADZONE
                || Math.abs(strafe.getAsDouble()) > JOYSTICK_DEADZONE
                || Math.abs(rotate.getAsDouble()) > JOYSTICK_DEADZONE;
    }
}