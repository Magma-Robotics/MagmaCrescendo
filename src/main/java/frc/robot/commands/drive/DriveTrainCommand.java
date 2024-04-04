package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveTrain;

public class DriveTrainCommand extends Command {
    private final DriveTrain driveTrain;
    private final CommandXboxController driveController;

    public DriveTrainCommand(DriveTrain driveTrain, CommandXboxController driveController) {
        this.driveController = driveController;
        this.driveTrain = driveTrain;
        addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        driveTrain.diffDriveJoystick(
            this.driveController.getLeftY()*.8, 
            this.driveController.getRightY()*.8);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
