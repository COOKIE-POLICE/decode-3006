package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Preferences;
import org.firstinspires.ftc.teamcode.modules.colorsensors.HsvColorSensor;

public class IndexerSubsystem extends SubsystemBase {
    private final CRServo indexerServo;

    public IndexerSubsystem(HardwareMap hardwareMap) {
        indexerServo = hardwareMap.get(CRServo.class, Preferences.INDEXER_SERVO);

    }
    public void indexRight() {
        indexerServo.setPower(1.0);
    }
    public void indexLeft() {
        indexerServo.setPower(-1.0);
    }
    public void stopIndex() {
        indexerServo.setPower(0);
    }
}
