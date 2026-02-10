package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.BlockerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import com.seattlesolvers.solverslib.command.CommandBase;

public class BlockCommand extends CommandBase {
    private final BlockerSubsystem blockerSubsystem;

    public BlockCommand(BlockerSubsystem blockerSubsystem) {
        this.blockerSubsystem = blockerSubsystem;
        addRequirements(this.blockerSubsystem);
    }

    @Override
    public void initialize() {
        blockerSubsystem.block();
    }


    @Override
    public boolean isFinished() {
        return true;
    }
}