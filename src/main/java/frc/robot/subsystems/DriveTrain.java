package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
    private CANSparkMax fLDrive, rLDrive, fRDrive, rRDrive;
    
    private RelativeEncoder fLDriveEncoder, fRDriveEncoder;

    private DifferentialDrive diffDrive;

    public DriveTrain() {
        fLDrive = new CANSparkMax(1, MotorType.kBrushless);
        rLDrive = new CANSparkMax(2, MotorType.kBrushless);
        fRDrive = new CANSparkMax(3, MotorType.kBrushless);
        rRDrive = new CANSparkMax(4, MotorType.kBrushless);

        fLDrive.restoreFactoryDefaults();
        rLDrive.restoreFactoryDefaults();
        fRDrive.restoreFactoryDefaults();
        rRDrive.restoreFactoryDefaults();

        rLDrive.follow(fLDrive);
        rRDrive.follow(fRDrive);

        this.fLDrive.setInverted(false);
        this.fRDrive.setInverted(true);

        fLDriveEncoder = fLDrive.getEncoder();
        fRDriveEncoder = fRDrive.getEncoder();

        fLDriveEncoder.setPositionConversionFactor(.2);
        fRDriveEncoder.setPositionConversionFactor(.2);

        fLDrive.burnFlash();
        rLDrive.burnFlash();
        fRDrive.burnFlash();
        rRDrive.burnFlash();

        diffDrive = new DifferentialDrive(fLDrive, fRDrive);

        resetEncoders();
    }

    public void diffDrive(double leftJoystick, double rightJoystick) {
        diffDrive.tankDrive(leftJoystick, rightJoystick);
    }

    public void diffDriveJoystick(double leftJoystick, double rightJoystick) {
        diffDrive.tankDrive(-leftJoystick, -rightJoystick);
    }

    public void stop() {
        fLDrive.stopMotor();
        fRDrive.stopMotor();
    }

    public void resetEncoders() {
        fLDriveEncoder.setPosition(0);
        fRDriveEncoder.setPosition(0);
    }

    public double getLeftEncoderPos() {
        return fLDriveEncoder.getPosition();
    }

    public double getRightEncoderPos() {
        return fRDriveEncoder.getPosition();
    }
}
