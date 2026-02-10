package org.firstinspires.ftc.teamcode.autonomous.twelveartifact;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Preferences;
import org.firstinspires.ftc.teamcode.autonomous.twelveartifact.BaseTwelveArtifactAutonomous;

@Autonomous
public class TwelveArtifactBlueCloseAutonomous extends BaseTwelveArtifactAutonomous {
    @Override
    protected Pose getStartingPose() {
        return Preferences.Poses.StartingPoses.BLUE_CLOSE_START_POSE;
    }

    @Override
    protected Pose getLaunchPose() {
        return Preferences.Poses.CloseLaunchingPoses.CLOSE_BLUE_LAUNCH_POSE;
    }

    @Override
    protected Pose getMovementPose() {
        return Preferences.Poses.MovementPoses.MOVEMENT_BLUE_FAR_POSE;
    }
    @Override
    protected Pose getGrabPoseOneStart() {
        return Preferences.Poses.BLUE_GRAB_POSE_ONE_START;
    }
    @Override
    protected Pose getGoalPose() {
        return Preferences.Poses.BLUE_GOAL_POSE;
    }
    @Override
    protected Pose getGrabPoseOneEnd() {
        return Preferences.Poses.BLUE_GRAB_POSE_ONE_END;
    }

    @Override
    protected Pose getGrabPoseTwoStart() {
        return Preferences.Poses.BLUE_GRAB_POSE_TWO_START;
    }
    @Override
    protected Pose getGrabPoseTwoEnd() {
        return Preferences.Poses.BLUE_GRAB_POSE_TWO_END;
    }
    @Override
    protected Pose getGrabPoseThreeStart() {
        return Preferences.Poses.BLUE_GRAB_POSE_THREE_START;
    }
    @Override
    protected Pose getGrabPoseThreeEnd() {
        return Preferences.Poses.BLUE_GRAB_POSE_THREE_END;
    }
}

