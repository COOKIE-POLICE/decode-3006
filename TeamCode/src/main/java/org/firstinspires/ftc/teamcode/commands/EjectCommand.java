package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import com.seattlesolvers.solverslib.command.CommandBase;

public class EjectCommand extends CommandBase {
    private final IntakeSubsystem intake;

    public EjectCommand(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(this.intake);
    }

    @Override
    public void initialize() {
        intake.startEjecting();
    }


    @Override
    public boolean isFinished() {
        return true;
    }
}