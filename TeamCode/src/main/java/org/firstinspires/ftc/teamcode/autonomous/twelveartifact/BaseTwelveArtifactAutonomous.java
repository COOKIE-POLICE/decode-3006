package org.firstinspires.ftc.teamcode.autonomous.twelveartifact;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;

import org.firstinspires.ftc.teamcode.commands.AdmitCommand;
import org.firstinspires.ftc.teamcode.commands.BlockCommand;
import org.firstinspires.ftc.teamcode.commands.GoToPoseCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.LaunchCommand;
import org.firstinspires.ftc.teamcode.commands.StopIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.StopLaunchCommand;
import org.firstinspires.ftc.teamcode.commands.TurnIndicatorGreenCommand;
import org.firstinspires.ftc.teamcode.commands.TurnIndicatorOffCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.BlockerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndicatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemFactory;

public abstract class BaseTwelveArtifactAutonomous extends CommandOpMode {

    protected abstract Pose getMovementPose();
    protected abstract Pose getStartingPose();
    protected abstract Pose getLaunchPose();
    protected abstract Pose getGoalPose();
    protected abstract Pose getGrabPoseOneStart();
    protected abstract Pose getGrabPoseOneEnd();
    protected abstract Pose getGrabPoseTwoStart();
    protected abstract Pose getGrabPoseTwoEnd();
    protected abstract Pose getGrabPoseThreeStart();
    protected abstract Pose getGrabPoseThreeEnd();
    protected abstract Pose getGoalRelease();

    private Follower follower;
    private LauncherSubsystem launcherSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private BlockerSubsystem blockerSubsystem;
    private IndicatorSubsystem indicatorSubsystem;

    @Override
    public void initialize() {
        super.reset();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(getStartingPose());

        intakeSubsystem = SubsystemFactory.createIntake(hardwareMap);
        indicatorSubsystem = SubsystemFactory.createIndicator(hardwareMap);
        blockerSubsystem = SubsystemFactory.createBlocker(hardwareMap);
        launcherSubsystem = SubsystemFactory.createLauncher(hardwareMap, follower, getGoalPose());

        Trigger atLaunchVelocityTrigger = new Trigger(() -> launcherSubsystem.isAtTargetVelocity());

        atLaunchVelocityTrigger
                .whileActiveContinuous(new TurnIndicatorGreenCommand(indicatorSubsystem));

        atLaunchVelocityTrigger
                .negate()
                .whileActiveContinuous(new TurnIndicatorOffCommand(indicatorSubsystem));

        schedule(new SequentialCommandGroup(
                launchArtifacts(),
                intakeRow(getGrabPoseOneStart(), getGrabPoseOneEnd()),
                launchArtifacts(),
                intakeRow(getGrabPoseTwoStart(), getGrabPoseTwoEnd()),
                launchArtifacts(),
                new GoToPoseCommand(follower, getGoalRelease()),
                intakeRow(getGrabPoseThreeStart(), getGrabPoseThreeEnd()),
                launchArtifacts(),
                new GoToPoseCommand(follower, getMovementPose())
        ));
    }

    private SequentialCommandGroup intakeRow(Pose grabPoseStart, Pose grabPoseEnd) {
        return new SequentialCommandGroup(
                new GoToPoseCommand(follower, grabPoseStart),
                new IntakeCommand(intakeSubsystem),
                new GoToPoseCommand(follower, grabPoseEnd),
                new StopIntakeCommand(intakeSubsystem)
        );
    }

    private SequentialCommandGroup launchArtifacts() {
        return new SequentialCommandGroup(
                new GoToPoseCommand(follower, getLaunchPose()),
                new LaunchCommand(launcherSubsystem),
                new AdmitCommand(blockerSubsystem),
                new WaitUntilCommand(() -> launcherSubsystem.isAtTargetVelocity()),
                new IntakeCommand(intakeSubsystem),
                new WaitCommand(500),
                new StopIntakeCommand(intakeSubsystem),
                new StopLaunchCommand(launcherSubsystem),
                new BlockCommand(blockerSubsystem)
        );
    }

    @Override
    public void run() {
        super.run();
        follower.update();
    }
}