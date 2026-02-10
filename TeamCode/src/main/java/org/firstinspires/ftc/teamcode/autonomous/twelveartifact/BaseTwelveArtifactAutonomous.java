package org.firstinspires.ftc.teamcode.autonomous.twelveartifact;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.Preferences;
import org.firstinspires.ftc.teamcode.commands.AdmitCommand;
import org.firstinspires.ftc.teamcode.commands.BlockCommand;
import org.firstinspires.ftc.teamcode.commands.GoToPoseCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.LaunchCommand;
import org.firstinspires.ftc.teamcode.commands.StopIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.StopLaunchCommand;
import org.firstinspires.ftc.teamcode.commands.TurnIndicatorGreenCommand;
import org.firstinspires.ftc.teamcode.commands.TurnIndicatorOffCommand;
import org.firstinspires.ftc.teamcode.commands.TurnToPoseCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.BlockerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndicatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;


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

    private Follower follower;
    TelemetryData telemetryData = new TelemetryData(telemetry);
    private LauncherSubsystem launcherSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private BlockerSubsystem blockerSubsystem;
    private Trigger atLaunchVelocityTrigger;
    private IndicatorSubsystem indicatorSubsystem;

    @Override
    public void initialize() {
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        indicatorSubsystem = new IndicatorSubsystem(hardwareMap);
        blockerSubsystem = new BlockerSubsystem(hardwareMap);

        super.reset();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(getStartingPose());
        launcherSubsystem = new LauncherSubsystem(hardwareMap, follower, Preferences.Poses.BLUE_GOAL_POSE);

        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(
                launchArtifacts(),
                intakeRow(getGrabPoseOneStart(), getGrabPoseOneEnd()),
                launchArtifacts(),
                intakeRow(getGrabPoseTwoStart(), getGrabPoseTwoEnd()),
                launchArtifacts(),
                intakeRow(getGrabPoseThreeStart(), getGrabPoseThreeEnd()),
                new GoToPoseCommand(follower, getMovementPose())








        );
        atLaunchVelocityTrigger = new Trigger(
                () -> launcherSubsystem.isAtTargetVelocity()
        );
        atLaunchVelocityTrigger
                .whileActiveContinuous(new TurnIndicatorGreenCommand(indicatorSubsystem));

        atLaunchVelocityTrigger
                .negate()
                .whileActiveContinuous(new TurnIndicatorOffCommand(indicatorSubsystem));
        schedule(autonomousSequence);
    }
    public SequentialCommandGroup intakeRow(Pose grabPoseStart, Pose grabPoseEnd) {
        return new SequentialCommandGroup(
                new GoToPoseCommand(follower, grabPoseStart),
                new GoToPoseCommand(follower, grabPoseStart),
                new IntakeCommand(intakeSubsystem),
                new GoToPoseCommand(follower, grabPoseEnd, 0.25),
                new GoToPoseCommand(follower, grabPoseEnd, 0.25),
                new WaitCommand(500),
                new StopIntakeCommand(intakeSubsystem)

        );

    }

    public SequentialCommandGroup launchArtifacts() {
        return new SequentialCommandGroup(
                new GoToPoseCommand(follower, getLaunchPose()),
                new GoToPoseCommand(follower, getLaunchPose()),
                new AdmitCommand(blockerSubsystem),
                new LaunchCommand(launcherSubsystem),
                new WaitUntilCommand(() -> launcherSubsystem.isAtTargetVelocity()),
                new WaitCommand(1000),
                new IntakeCommand(intakeSubsystem),
                new WaitCommand(1000),
                new StopIntakeCommand(intakeSubsystem),
                new StopLaunchCommand(launcherSubsystem),
                new BlockCommand(blockerSubsystem)
        );
    }

    @Override
    public void run() {
        super.run();
        follower.update();
        /*

        telemetryData.addData("X", follower.getPose().getX());
        telemetryData.addData("Y", follower.getPose().getY());
        telemetryData.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Current Velocity", "%.2f", launcherSubsystem.getVelocityMagnitude());
        telemetry.addData("Target Velocity", "%.2f", launcherSubsystem.getTargetVelocity());
        telemetry.addData("Velocity Error", "%.2f", launcherSubsystem.getVelocityError());
        telemetry.addData("Percent Error", "%.2f%%", launcherSubsystem.getPercentError());
        telemetryData.update();
         */
    }
}
