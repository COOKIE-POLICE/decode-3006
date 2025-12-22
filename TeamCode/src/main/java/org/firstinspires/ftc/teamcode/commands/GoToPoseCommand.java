package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.CommandBase;

public class GoToPoseCommand extends CommandBase {
    private final Follower follower;
    private final Pose targetPose;
    private final boolean holdEnd;
    private final double maxPower;
    private PathChain pathChain;

    public GoToPoseCommand(Follower follower, Pose targetPose) {
        this(follower, targetPose, true, 1.0);
    }

    public GoToPoseCommand(Follower follower, Pose targetPose, boolean holdEnd) {
        this(follower, targetPose, holdEnd, 1.0);
    }

    public GoToPoseCommand(Follower follower, Pose targetPose, double maxPower) {
        this(follower, targetPose, true, maxPower);
    }

    public GoToPoseCommand(Follower follower, Pose targetPose, boolean holdEnd, double maxPower) {
        this.follower = follower;
        this.targetPose = targetPose;
        this.holdEnd = holdEnd;
        this.maxPower = maxPower;
    }

    public GoToPoseCommand setGlobalMaxPower(double globalMaxPower) {
        follower.setMaxPower(globalMaxPower);
        return this;
    }

    @Override
    public void initialize() {
        Pose currentPose = follower.getPose();

        pathChain = follower.pathBuilder()
                .addPath(new BezierLine(currentPose, targetPose))
                .setLinearHeadingInterpolation(currentPose.getHeading(), targetPose.getHeading())
                .build();

        follower.followPath(pathChain, maxPower, holdEnd);
    }

    @Override
    public boolean isFinished() {
        return !follower.isBusy();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            follower.breakFollowing();
            follower.startTeleopDrive();
        }
    }
}