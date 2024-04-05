package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private CANSparkMax leftShooter, rightShooter;

    private RelativeEncoder leftEncoder, rightEncoder;

    private SimpleMotorFeedforward leftFeedforward= new SimpleMotorFeedforward(
        Constants.Subsystems.Shooter.kLeftS, Constants.Subsystems.Shooter.kLeftV, Constants.Subsystems.Shooter.kLeftA);
    
    private SimpleMotorFeedforward rightFeedforward= new SimpleMotorFeedforward(
        Constants.Subsystems.Shooter.kRightS, Constants.Subsystems.Shooter.kRightV, Constants.Subsystems.Shooter.kRightA);

    public Shooter() {
        leftShooter = new CANSparkMax(7, MotorType.kBrushless);
        rightShooter = new CANSparkMax(8, MotorType.kBrushless);

        leftEncoder = leftShooter.getEncoder();
        rightEncoder = rightShooter.getEncoder();

        leftEncoder.setPositionConversionFactor(Units.inchesToMeters(4) * Math.PI);
        rightEncoder.setPositionConversionFactor(Units.inchesToMeters(4) * Math.PI);
        leftEncoder.setVelocityConversionFactor(Units.inchesToMeters(4) * Math.PI / 60);
        rightEncoder.setVelocityConversionFactor(Units.inchesToMeters(4) * Math.PI / 60);

        leftShooter.restoreFactoryDefaults();
        rightShooter.restoreFactoryDefaults();

        //rightShooter.follow(leftShooter, true);
        leftShooter.setInverted(true);
        rightShooter.setInverted(false);

        leftShooter.burnFlash();
        rightShooter.burnFlash();
    }

    public void TargetFeedforward(double velocity) {
        leftShooter.setVoltage(leftFeedforward.calculate(velocity));
        rightShooter.setVoltage(rightFeedforward.calculate(velocity));
      }

    public void SetShooter(double power) {
        leftShooter.set(power);
        rightShooter.set(power);
    }

    public void ShooterStop() {
        leftShooter.stopMotor();
        rightShooter.stopMotor();
    }
    
}
