package org.firstinspires.ftc.teamcode.teleops.normal;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Preferences;
import org.firstinspires.ftc.teamcode.teleops.BaseTeleOp;

@Configurable
@TeleOp(name = "Red Close TeleOp", group = "TeleOp")
public class RedCloseTeleOp extends BaseTeleOp {
    @Override
    protected Pose getStartingPose() {
        return Preferences.Poses.StartingPoses.RED_CLOSE_START_POSE;
    }

    @Override
    protected Pose getGoalPose() {
        return Preferences.Poses.RED_GOAL_POSE;
    }

    @Override
    protected Pose getLaunchPose() {
        return Preferences.Poses.LaunchingPoses.RED_LAUNCH_POSE;
    }

    @Override
    protected Pose getCloseLaunchPose() {
        return Preferences.Poses.CloseLaunchingPoses.CLOSE_RED_LAUNCH_POSE;
    }

    @Override
    protected Pose getGateReleasePose() {
        return Preferences.Poses.RED_GATE_RELEASE_POSE;
    }

    @Override
    protected Pose getParkPose() {
        return Preferences.Poses.RED_PARK_POSE;
    }
}