package org.firstinspires.ftc.teamcode.autonomous.threeartifact;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Preferences;

@Autonomous
public class ThreeArtifactBlueFarAutonomous extends BaseThreeArtifactAutonomous {
    @Override
    protected Pose getStartingPose() {
        return Preferences.Poses.StartingPoses.BLUE_FAR_START_POSE;
    }

    @Override
    protected Pose getLaunchPose() {
        return Preferences.Poses.LaunchingPoses.BLUE_LAUNCH_POSE;
    }

    @Override
    protected Pose getMovementPose() {
        return Preferences.Poses.MovementPoses.MOVEMENT_BLUE_FAR_POSE;
    }
}

