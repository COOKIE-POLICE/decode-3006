package org.firstinspires.ftc.teamcode.autonomous.movement;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Preferences;

@Autonomous
public class MovementBlueFarAutonomous extends BaseMovementAutonomous {
    @Override
    protected Pose getStartingPose() {
        return Preferences.Poses.StartingPoses.BLUE_FAR_START_POSE;
    }
    @Override
    protected Pose getMovementPose() { return Preferences.Poses.MovementPoses.MOVEMENT_BLUE_FAR_POSE; }

}