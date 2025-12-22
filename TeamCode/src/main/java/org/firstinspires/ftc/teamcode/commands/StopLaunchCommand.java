package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;

public class StopLaunchCommand extends CommandBase {
    private final LauncherSubsystem launcher;
    public StopLaunchCommand(LauncherSubsystem launcher) {
        this.launcher = launcher;
        addRequirements(launcher);

    }

    @Override
    public void initialize() {
        launcher.stopLaunching();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}