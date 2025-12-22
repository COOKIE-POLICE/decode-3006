package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.pedroCommand.TurnCommand;
import org.firstinspires.ftc.teamcode.Preferences;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;

public class TurnToRedGoalCommand extends CommandBase {
    private final Follower follower;
    private final LimelightSubsystem limelight;
    private final double alignmentTolerance;
    private CommandBase currentCommand;
    private boolean usedVision;

    public TurnToRedGoalCommand(
            Follower follower,
            LimelightSubsystem limelight
    ) {
        this(follower, limelight, 1.0);
    }

    public TurnToRedGoalCommand(
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
        limelight.switchPipeline(Preferences.Limelight.PIPELINE_RED_GOAL);
        usedVision = limelight.hasValidTarget();

        if (usedVision) {
            double targetX = limelight.getTargetX();
            currentCommand = new TurnCommand(
                    follower,
                    Math.toRadians(-targetX + 180),
                    targetX < 0
            );
            currentCommand.initialize();
        } else {
            currentCommand = new TurnToPoseCommand(
                    follower,
                    Preferences.Poses.RED_GOAL_POSE
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
            double targetX = limelight.getTargetX();
            if (Math.abs(targetX) > alignmentTolerance && !follower.isBusy()) {
                currentCommand = new TurnCommand(
                        follower,
                        Math.toRadians(-targetX),
                        targetX < 0
                );
                currentCommand.initialize();
            }
        }
    }

    @Override
    public boolean isFinished() {
        if (!usedVision && currentCommand != null) {
            return currentCommand.isFinished();
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if (currentCommand != null) {
            currentCommand.end(interrupted);
        }
    }
}