package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    private CANSparkMax Indexer = new CANSparkMax(9, MotorType.kBrushless);

    public Indexer() {
        Indexer.restoreFactoryDefaults();
        Indexer.setInverted(false);
        Indexer.burnFlash();
    }

    public void SetIndexer(double power) {
        Indexer.set(power);
    }

    public void IndexerStop() {
        this.Indexer.stopMotor();
    }

}
