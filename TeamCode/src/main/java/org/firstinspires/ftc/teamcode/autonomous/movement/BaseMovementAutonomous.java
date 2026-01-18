package org.firstinspires.ftc.teamcode.autonomous.movement;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.Preferences;
import org.firstinspires.ftc.teamcode.commands.GoToPoseCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public abstract class BaseMovementAutonomous extends CommandOpMode {
    protected abstract Pose getMovementPose();
    protected abstract Pose getStartingPose();
    private Follower follower;
    TelemetryData telemetryData = new TelemetryData(telemetry);

    @Override
    public void initialize() {

        super.reset();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(getStartingPose());

        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(
                new GoToPoseCommand(follower, getMovementPose())
        );
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
        telemetryData.update();
         */
    }
}
