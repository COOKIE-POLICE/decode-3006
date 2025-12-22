package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.HoodLifterSubsystem;

public class HoodDownCommand extends InstantCommand {
    public HoodDownCommand(HoodLifterSubsystem hood) {
        super(hood::setDown, hood);
    }
}