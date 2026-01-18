package org.firstinspires.ftc.teamcode.teleops.normal;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Preferences;
import org.firstinspires.ftc.teamcode.teleops.BaseTeleOp;

@Configurable
@TeleOp(name = "Blue Far TeleOp", group = "TeleOp")
public class BlueFarTeleOp extends BaseTeleOp {
    @Override
    protected Pose getStartingPose() {
        return Preferences.Poses.StartingPoses.BLUE_FAR_START_POSE;
    }

    @Override
    protected Pose getGoalPose() {
        return Preferences.Poses.BLUE_GOAL_POSE;
    }

    @Override
    protected Pose getLaunchPose() {
        return Preferences.Poses.LaunchingPoses.BLUE_LAUNCH_POSE;
    }

    @Override
    protected Pose getCloseLaunchPose() {
        return Preferences.Poses.CloseLaunchingPoses.CLOSE_BLUE_LAUNCH_POSE;
    }

    @Override
    protected Pose getGateReleasePose() {
        return Preferences.Poses.BLUE_GATE_RELEASE_POSE;
    }

    @Override
    protected Pose getParkPose() {
        return Preferences.Poses.BLUE_PARK_POSE;
    }
}