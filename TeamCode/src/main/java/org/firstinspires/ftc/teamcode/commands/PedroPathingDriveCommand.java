package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.command.CommandBase;
import java.util.function.DoubleSupplier;

public class PedroPathingDriveCommand extends CommandBase {
    private final Follower follower;
    private final DoubleSupplier forward;
    private final DoubleSupplier strafe;
    private final DoubleSupplier rotate;

    public PedroPathingDriveCommand(
            Follower follower,
            DoubleSupplier forward,
            DoubleSupplier strafe,
            DoubleSupplier rotate
    ) {
        this.follower = follower;
        this.forward = forward;
        this.strafe = strafe;
        this.rotate = rotate;
    }

    @Override
    public void execute() {
        follower.setTeleOpDrive(
                forward.getAsDouble(),
                strafe.getAsDouble(),
                rotate.getAsDouble(),
                true
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}