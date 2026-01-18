package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Preferences;

public class IndicatorSubsystem extends SubsystemBase {
    private Servo indicator;
    private static final double GREEN = 0.500;
    private static final double OFF = 0.0;


    public IndicatorSubsystem(HardwareMap hardwareMap) {
        indicator = hardwareMap.get(Servo.class, "indicator");
    }

    public void turnGreen() {
        indicator.setPosition(GREEN);
    }

    public void turnOff() {
        indicator.setPosition(OFF);
    }
}
