package org.firstinspires.ftc.teamcode.autonomous.sixartifact;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Preferences;
@Autonomous
@Disabled
public class SixArtifactRedFarAutonomous extends BaseSixArtifactAutonomous {
    @Override
    protected Pose getStartingPose() {
        return Preferences.Poses.StartingPoses.RED_FAR_START_POSE;
    }

    @Override
    protected Pose getLaunchPose() {
        return Preferences.Poses.LaunchingPoses.RED_LAUNCH_POSE;
    }

    @Override
    protected Pose getMovementPose() {
        return Preferences.Poses.MovementPoses.MOVEMENT_RED_FAR_POSE;
    }
    @Override
    protected Pose getHumanPlayerGrabStart() {
        return Preferences.Poses.RED_HUMAN_PLAYER_GRAB_POSE_START;
    }
    @Override
    protected Pose getHumanPlayerGrabEnd() {
        return Preferences.Poses.RED_HUMAN_PLAYER_GRAB_POSE_END;
    }
}

