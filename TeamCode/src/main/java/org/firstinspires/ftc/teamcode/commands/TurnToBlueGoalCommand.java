package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.pedroCommand.TurnCommand;
import org.firstinspires.ftc.teamcode.Preferences;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;

public class TurnToBlueGoalCommand extends CommandBase {
    private final Follower follower;
    private final LimelightSubsystem limelight;
    private final double alignmentTolerance;
    private CommandBase currentCommand;
    private boolean usedVision;

    public TurnToBlueGoalCommand(
            Follower follower,
            LimelightSubsystem limelight
    ) {
        this(follower, limelight, 1.0);
    }

    public TurnToBlueGoalCommand(
            Follower follower,
            LimelightSubsystem limelight,
            double alignmentTolerance
    ) {
        this.follower = follower;
        this.limelight = limelight;
        this.alignmentTolerance = alignmentTolerance;
    }

    @Override
    public void initialize() {
        limelight.switchPipeline(Preferences.Limelight.PIPELINE_BLUE_GOAL);
        usedVision = limelight.hasValidTarget();

        if (usedVision) {
            double targetXDegrees = limelight.getTargetX();
            currentCommand = new TurnCommand(
                    follower,
                    Math.toRadians(-targetXDegrees),
                    targetXDegrees < 0
            );
            currentCommand.initialize();
        } else {
            currentCommand = new TurnToPoseCommand(
                    follower,
                    Preferences.Poses.BLUE_GOAL_POSE
            );
            currentCommand.initialize();
        }
    }

    @Override
    public void execute() {
        if (currentCommand != null) {
            currentCommand.execute();
        }

        if (usedVision && limelight.hasValidTarget()) {
            double targetXDegrees = limelight.getTargetX();
            if (Math.abs(targetXDegrees) > alignmentTolerance && !follower.isBusy()) {
                currentCommand = new TurnCommand(
                        follower,
                        Math.toRadians(-targetXDegrees + 180),
                        targetXDegrees < 0
                );
                currentCommand.initialize();
            }
        } else if (!usedVision && !follower.isBusy()) {
            currentCommand = new TurnToPoseCommand(
                    follower,
                    Preferences.Poses.BLUE_GOAL_POSE
            );
            currentCommand.initialize();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if (currentCommand != null) {
            currentCommand.end(interrupted);
        }
    }
}