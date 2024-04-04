package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class DriveEncoders extends Command {
    private DriveTrain driveTrain;
    private double driveDistance;
    private double botSpeed;
    private double endDistance;
    private boolean reverse;

    public DriveEncoders(DriveTrain driveTrain, double speed, double userFeet, boolean reverse) {
        driveDistance = userFeet;
        endDistance = userFeet;
        botSpeed = speed;
        this.reverse = reverse;
        this.driveTrain = driveTrain;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
      driveTrain.resetEncoders();
      if (reverse != true) {
        endDistance = driveDistance;
      }
      else {
        endDistance = -driveDistance;
      }
    }
    @Override
    public void execute() {
      driveTrain.diffDrive(botSpeed, botSpeed);
      SmartDashboard.putNumber("Left Encoder Pos", driveTrain.getLeftEncoderPos());
    }

    @Override
    public boolean isFinished() {
      if (reverse != true) {
        return (driveTrain.getLeftEncoderPos() >= endDistance);
      }
      else {
        return (driveTrain.getLeftEncoderPos() <= endDistance);
      }
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.stop();
    }
}
