package org.firstinspires.ftc.teamcode.autonomous.sixartifact;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.Preferences;
import org.firstinspires.ftc.teamcode.commands.GoToPoseCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.LaunchCommand;
import org.firstinspires.ftc.teamcode.commands.StopIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.StopLaunchCommand;
import org.firstinspires.ftc.teamcode.commands.TurnIndicatorGreenCommand;
import org.firstinspires.ftc.teamcode.commands.TurnIndicatorOffCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IndicatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;


public abstract class BaseSixArtifactAutonomous extends CommandOpMode {
    protected abstract Pose getMovementPose();
    protected abstract Pose getStartingPose();
    protected abstract Pose getLaunchPose();
    protected abstract Pose getHumanPlayerGrabStart();
    protected abstract Pose getHumanPlayerGrabEnd();

    private Follower follower;
    TelemetryData telemetryData = new TelemetryData(telemetry);
    private LauncherSubsystem launcherSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private Trigger atLaunchVelocityTrigger;
    private IndicatorSubsystem indicatorSubsystem;

    @Override
    public void initialize() {
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        indicatorSubsystem = new IndicatorSubsystem(hardwareMap);

        super.reset();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(getStartingPose());
        launcherSubsystem = new LauncherSubsystem(hardwareMap, follower, Preferences.Poses.BLUE_GOAL_POSE);

        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(

                new GoToPoseCommand(follower, getLaunchPose()),
                new GoToPoseCommand(follower, getLaunchPose()),
                new LaunchCommand(launcherSubsystem),
                new WaitUntilCommand(() -> launcherSubsystem.isAtTargetVelocity()),
                new WaitCommand(1000),
                new IntakeCommand(intakeSubsystem),
                new WaitCommand(3000),
                new WaitCommand(3000),
                new WaitCommand(3000),
                new StopLaunchCommand(launcherSubsystem),
                new StopIntakeCommand(intakeSubsystem),

                new GoToPoseCommand(follower, getHumanPlayerGrabStart()),
                new IntakeCommand(intakeSubsystem),
                new GoToPoseCommand(follower, getHumanPlayerGrabEnd(), 0.45),
                new WaitCommand(500),
                new StopIntakeCommand(intakeSubsystem),

                new GoToPoseCommand(follower, getLaunchPose()),
                new GoToPoseCommand(follower, getLaunchPose()),
                new LaunchCommand(launcherSubsystem),
                new WaitUntilCommand(() -> launcherSubsystem.isAtTargetVelocity()),
                new WaitCommand(1000),
                new IntakeCommand(intakeSubsystem),
                new WaitCommand(3000),
                new WaitCommand(3000),
                new WaitCommand(3000),
                new StopLaunchCommand(launcherSubsystem),
                new StopIntakeCommand(intakeSubsystem),


                new GoToPoseCommand(follower, getMovementPose()),
                new StopLaunchCommand(launcherSubsystem),
                new StopIntakeCommand(intakeSubsystem)

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
