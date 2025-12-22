package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;

public class LaunchCommand extends CommandBase {
    private final LauncherSubsystem launcher;
    public LaunchCommand(LauncherSubsystem launcher) {
        this.launcher = launcher;
        addRequirements(launcher);

    }

    @Override
    public void initialize() {
        launcher.startLaunching();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}