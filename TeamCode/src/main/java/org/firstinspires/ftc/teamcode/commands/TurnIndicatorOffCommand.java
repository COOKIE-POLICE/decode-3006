package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IndicatorSubsystem;

public class TurnIndicatorOffCommand extends CommandBase {
    private final IndicatorSubsystem indicatorSubsystem;
    public TurnIndicatorOffCommand(IndicatorSubsystem indicator) {
        this.indicatorSubsystem = indicator;
        addRequirements(this.indicatorSubsystem);
    }

    @Override
    public void initialize() {
        indicatorSubsystem.turnOff();
    }


    @Override
    public boolean isFinished() {
        return true;
    }
}
