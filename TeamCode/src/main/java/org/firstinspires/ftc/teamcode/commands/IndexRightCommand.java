package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;

public class IndexRightCommand extends CommandBase {
    private final IndexerSubsystem indexer;

    public IndexRightCommand(IndexerSubsystem indexer) {
        this.indexer = indexer;
        addRequirements(this.indexer);
    }

    @Override
    public void initialize() {
        indexer.indexRight();
    }


    @Override
    public boolean isFinished() {
        return true;
    }
}