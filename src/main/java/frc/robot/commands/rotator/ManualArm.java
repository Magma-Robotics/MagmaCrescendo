package frc.robot.commands.rotator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Rotator;

public class ManualArm extends Command {
    private double power;
    private Rotator rotator;

    public ManualArm(Rotator rotator, double power) {
        this.power = power;
        this.rotator = rotator;
        addRequirements(rotator);
    }

    public void execute() {
        SmartDashboard.putNumber("Arm pos", rotator.getLeftEncoderPos());
        rotator.ManualArm(power);
    }

    public boolean isFinished() {
        return false;
    }
}
