package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.Preferences;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;

import java.util.function.DoubleSupplier;
public class BlueGoalLockedOnDriveCommand extends CommandBase {
    private final Follower follower;
    private final DoubleSupplier forward;
    private final DoubleSupplier strafe;
    private final LimelightSubsystem limelight;
    private final PIDFController pidfController;

    private static final double LIMELIGHT_HEADING_OFFSET = Math.PI;

    public BlueGoalLockedOnDriveCommand(
            Follower follower,
            DoubleSupplier forward,
            DoubleSupplier strafe,
            LimelightSubsystem limelight
    ) {
        this(follower, forward, strafe, limelight, 1.0, 0.02, 0.0, 0.001, 0.0);
    }

    public BlueGoalLockedOnDriveCommand(
            Follower follower,
            DoubleSupplier forward,
            DoubleSupplier strafe,
            LimelightSubsystem limelight,
            double alignmentTolerance,
            double proportional,
            double integral,
            double derivative,
            double feedforward
    ) {
        this.follower = follower;
        this.forward = forward;
        this.strafe = strafe;
        this.limelight = limelight;
        this.pidfController = new PIDFController(proportional, integral, derivative, feedforward);
        this.pidfController.setTolerance(alignmentTolerance);
    }

    @Override
    public void initialize() {
        limelight.switchPipeline(Preferences.Limelight.PIPELINE_BLUE_GOAL);
        pidfController.reset();
    }

    @Override
    public void execute() {
        double rotationOutput;

        if (limelight.hasValidTarget()) {
            pidfController.setSetPoint(0.0);
            rotationOutput = pidfController.calculate(limelight.getTargetX());
        } else {
            Pose currentPose = follower.getPose();
            Pose goalPose = Preferences.Poses.BLUE_GOAL_POSE;

            double deltaX = goalPose.getX() - currentPose.getX();
            double deltaY = goalPose.getY() - currentPose.getY();
            double targetHeading = Math.atan2(deltaY, deltaX) + LIMELIGHT_HEADING_OFFSET;

            while (targetHeading > Math.PI) targetHeading -= 2 * Math.PI;
            while (targetHeading < -Math.PI) targetHeading += 2 * Math.PI;

            pidfController.setSetPoint(targetHeading);
            rotationOutput = pidfController.calculate(currentPose.getHeading());
        }

        follower.setTeleOpDrive(
                forward.getAsDouble(),
                strafe.getAsDouble(),
                rotationOutput,
                true
        );
    }

    public boolean isAligned() {
        return pidfController.atSetPoint();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        pidfController.reset();
    }
}