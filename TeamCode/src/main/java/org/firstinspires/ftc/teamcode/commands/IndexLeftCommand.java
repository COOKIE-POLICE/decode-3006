package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;

public class IndexLeftCommand extends CommandBase {
    private final IndexerSubsystem indexer;

    public IndexLeftCommand(IndexerSubsystem indexer) {
        this.indexer = indexer;
        addRequirements(this.indexer);
    }

    @Override
    public void initialize() {
        indexer.indexLeft();
    }


    @Override
    public boolean isFinished() {
        return true;
    }
}