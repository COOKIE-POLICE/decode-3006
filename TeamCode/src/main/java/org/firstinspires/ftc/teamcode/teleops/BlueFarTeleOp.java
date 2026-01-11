package org.firstinspires.ftc.teamcode.teleops;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Preferences;

@Configurable
@TeleOp(name = "Blue Far TeleOp", group = "TeleOp")
public class BlueFarTeleOp extends BaseTeleOp {
    @Override
    protected Pose getStartingPose() {
        return Preferences.Poses.BLUE_FAR_START_POSE;
    }

    @Override
    protected Pose getGoalPose() {
        return Preferences.Poses.BLUE_GOAL_POSE;
    }

    @Override
    protected Pose getLaunchPose() {
        return Preferences.Poses.BLUE_LAUNCH_POSE;
    }

    @Override
    protected Pose getCloseLaunchPose() {
        return Preferences.Poses.CLOSE_BLUE_LAUNCH_POSE;
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