package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;

public class StopIntakeCommand extends CommandBase {
    private final IntakeSubsystem intake;

    public StopIntakeCommand(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(this.intake);
    }

    @Override
    public void initialize() {
        intake.stop();
    }


    @Override
    public boolean isFinished() {
        return true;
    }
}