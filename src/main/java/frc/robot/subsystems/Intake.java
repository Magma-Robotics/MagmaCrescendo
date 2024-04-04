package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private CANSparkMax Intake = new CANSparkMax(10, MotorType.kBrushless);

    public Intake() {
        Intake.restoreFactoryDefaults();
        Intake.burnFlash();
    }

    public void SetIntake(double power) {
        this.Intake.set(power);
    }

    public void IntakeStop() {
        this.Intake.set(0);
    }
}
