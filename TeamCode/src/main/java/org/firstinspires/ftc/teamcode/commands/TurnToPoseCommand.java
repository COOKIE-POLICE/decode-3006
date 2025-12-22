package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.CommandBase;

public class TurnToPoseCommand extends CommandBase {
    private final Follower follower;
    private final Pose targetPose;

    public TurnToPoseCommand(Follower follower, Pose targetPose) {
        this.follower = follower;
        this.targetPose = targetPose;
    }

    @Override
    public void initialize() {
        Pose currentPose = follower.getPose();
        double deltaX = targetPose.getX() - currentPose.getX();
        double deltaY = targetPose.getY() - currentPose.getY();
        double angleToTarget = Math.atan2(deltaY, deltaX);
        follower.turnTo(angleToTarget);
    }

    @Override
    public boolean isFinished() {
        return !follower.isBusy();
    }
}