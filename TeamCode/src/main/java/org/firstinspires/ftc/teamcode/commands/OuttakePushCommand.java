package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.OuttakerSubsystem;

public class OuttakePushCommand extends CommandBase {
    private final OuttakerSubsystem outtaker;
    public OuttakePushCommand(OuttakerSubsystem outtaker) {
        this.outtaker = outtaker;
        addRequirements(outtaker);

    }

    @Override
    public void initialize() {
        outtaker.push();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}