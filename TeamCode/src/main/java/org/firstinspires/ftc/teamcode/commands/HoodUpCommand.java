package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.HoodLifterSubsystem;

public class HoodUpCommand extends InstantCommand {
    public HoodUpCommand(HoodLifterSubsystem hood) {
        super(hood::setUp, hood);
    }
}