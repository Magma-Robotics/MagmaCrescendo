package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;


public class Rotate extends Command {
    private DriveTrain driveTrain;
    private double botSpeed;
    private double endDegrees;


    public Rotate(DriveTrain driveTrain, double botSpeed, double endDegrees) {
        this.endDegrees = endDegrees/55;
        this.botSpeed = botSpeed;
        this.driveTrain = driveTrain;
        addRequirements(driveTrain);
    }


    @Override
    public void initialize() {
      driveTrain.resetEncoders();
      if (botSpeed > 0) {
      }
      else {
        endDegrees = -endDegrees;
      }
    }
    @Override
    public void execute() {
      driveTrain.diffDrive(-botSpeed, botSpeed);
    }


    @Override
    public boolean isFinished() {
      if (botSpeed > 0) {
        return (driveTrain.getRightEncoderPos() >= endDegrees);
      }
      else {
        return (driveTrain.getRightEncoderPos() <= endDegrees);
      }
    }


    @Override
    public void end(boolean interrupted) {
        driveTrain.stop();
    }
}


