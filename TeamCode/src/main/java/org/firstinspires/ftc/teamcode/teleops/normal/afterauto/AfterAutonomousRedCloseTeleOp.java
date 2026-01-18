package org.firstinspires.ftc.teamcode.teleops.normal.afterauto;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Preferences;
import org.firstinspires.ftc.teamcode.teleops.normal.RedCloseTeleOp;

@TeleOp
public class AfterAutonomousRedCloseTeleOp extends RedCloseTeleOp {
    @Override
    protected Pose getStartingPose() {
        return Preferences.Poses.MovementPoses.MOVEMENT_RED_FAR_POSE;
    }
}
