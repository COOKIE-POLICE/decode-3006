package org.firstinspires.ftc.teamcode.teleops.normal.afterauto;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Preferences;
import org.firstinspires.ftc.teamcode.teleops.normal.BlueFarTeleOp;

@TeleOp
public class AfterAutonomousBlueFarTeleOp extends BlueFarTeleOp {
    @Override
    protected Pose getStartingPose() {
        return Preferences.Poses.MovementPoses.MOVEMENT_BLUE_FAR_POSE;
    }
}
