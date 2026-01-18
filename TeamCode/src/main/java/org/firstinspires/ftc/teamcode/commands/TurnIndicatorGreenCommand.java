package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IndicatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class TurnIndicatorGreenCommand extends CommandBase {
    private final IndicatorSubsystem indicatorSubsystem;
    public TurnIndicatorGreenCommand(IndicatorSubsystem indicator) {
        this.indicatorSubsystem = indicator;
        addRequirements(this.indicatorSubsystem);
    }

    @Override
    public void initialize() {
        indicatorSubsystem.turnGreen();
    }


    @Override
    public boolean isFinished() {
        return true;
    }
}
