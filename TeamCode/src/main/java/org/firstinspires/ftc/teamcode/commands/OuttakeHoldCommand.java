package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.OuttakerSubsystem;

public class OuttakeHoldCommand extends CommandBase {
    private final OuttakerSubsystem outtaker;
    public OuttakeHoldCommand(OuttakerSubsystem outtaker) {
        this.outtaker = outtaker;
        addRequirements(outtaker);

    }

    @Override
    public void initialize() {
        outtaker.hold();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}