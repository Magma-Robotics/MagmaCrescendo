// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.autos.DriveEncoders;
import frc.robot.commands.drive.DriveTrainCommand;
import frc.robot.commands.drive.DriveTrainCommandSlower;
import frc.robot.commands.indexer.IndexerStop;
import frc.robot.commands.indexer.SetIndexer;
import frc.robot.commands.intake.IntakeStop;
import frc.robot.commands.intake.SetIntake;
import frc.robot.commands.shooter.SetShooter;
import frc.robot.commands.shooter.ShooterStop;
import frc.robot.commands.shooter.TargetFeedforward;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Rotator;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private DriveTrain driveTrain = new DriveTrain();
  private Shooter Shooter = new Shooter();
  private Intake Intake = new Intake();
  private Indexer Indexer = new Indexer();
  private Rotator Rotator = new Rotator();

  private final CommandXboxController driverController =
    new CommandXboxController(0);
  private final CommandXboxController driverPartnerController =
    new CommandXboxController(1);
  private final CommandXboxController testController = 
    new CommandXboxController(2);
  SendableChooser<Command> m_auto_chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    SmartDashboard.putData("Auto Chooser", m_auto_chooser);
    m_auto_chooser.addOption("Mid Auto", MidAuto());

    driveTrain.setDefaultCommand(new DriveTrainCommand(driveTrain, driverController));

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    driverController
      .rightBumper().whileTrue(new DriveTrainCommandSlower(driveTrain, driverController));

    driverPartnerController
      .a().onTrue(Rotator.setArmGoalCommand(Constants.ArmConstants.kHome));
    driverPartnerController
      .b().onTrue(autoAmp());
    driverPartnerController
      .x().onTrue(Rotator.setArmGoalCommand(Constants.ArmConstants.kAmp));
    driverPartnerController
      .y().onTrue(autoShoot());
    driverPartnerController
      .rightBumper().onTrue(new SetShooter(Shooter, -Constants.Subsystems.Shooter.kPOWER)).onFalse(new ShooterStop(Shooter));
    driverPartnerController
      .rightBumper().onTrue(new SetIntake(Intake, Constants.Subsystems.Intake.kPOWER)).onFalse(new IntakeStop(Intake));
    driverPartnerController
      .leftBumper().onTrue(new SetIntake(Intake, -Constants.Subsystems.Intake.kPOWER)).onFalse(new IntakeStop(Intake));
    driverPartnerController
      .a().and(driverPartnerController.povUp()).onTrue(new ShooterStop(Shooter));
    driverPartnerController
      .a().and(driverPartnerController.povUp()).onTrue(new IndexerStop(Indexer));    

    testController
      .a().onTrue(Rotator.ManualArm(0.2)).onFalse(Rotator.StopArm());
    testController
      .b().onTrue(Rotator.ManualArm(-0.2)).onFalse(Rotator.StopArm());
    testController
      .y().onTrue(Rotator.ResetArmEncoder());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_auto_chooser.getSelected();
  }

  public Command autoAmp() {
    return new SequentialCommandGroup(
      new ParallelRaceGroup(
        new SetIndexer(Indexer, Constants.Subsystems.Indexer.kPOWER),
        new WaitCommand(0.5)),
      new IndexerStop(Indexer),
      new ParallelRaceGroup(
        new TargetFeedforward(Shooter, 4.2),
        new WaitCommand(1)
      ),
      new ShooterStop(Shooter)
    );
  }
  public Command autoShoot() {
    return new SequentialCommandGroup(
      Rotator.setArmGoalCommand(Constants.ArmConstants.kSpeaker),
      new ParallelRaceGroup(
        new TargetFeedforward(Shooter, 4.2),
        new WaitCommand(1)
      ),
      new ParallelRaceGroup(
        new SetIndexer(Indexer, Constants.Subsystems.Indexer.kPOWER),
        new WaitCommand(0.5)),
      new ShooterStop(Shooter),
      new IndexerStop(Indexer)
    );
  }
  
  public Command autoBackwardsShoot() {
    return new SequentialCommandGroup(
      Rotator.setArmGoalCommand(Constants.ArmConstants.kBackwardsSpeaker),
      new ParallelRaceGroup(
        new TargetFeedforward(Shooter, 4.2),
        new WaitCommand(1)
      ),
      new ParallelRaceGroup(
        new SetIndexer(Indexer, Constants.Subsystems.Indexer.kPOWER),
        new WaitCommand(0.5)),
      new ShooterStop(Shooter),
      new IndexerStop(Indexer)
    );
  }

  public Command MidAuto() {
    return new SequentialCommandGroup(
      //Rotate arm and shoot
      autoBackwardsShoot(),
      Rotator.setArmGoalCommand(Constants.ArmConstants.kHome),
      //Drive forward
      new DriveEncoders(driveTrain, 0.5, 4, false),
      //Intake
      new ParallelRaceGroup(
        new SetIntake(Intake, Constants.Subsystems.Intake.kPOWER),
        new SetShooter(Shooter, -Constants.Subsystems.Shooter.kPOWER),
        new WaitCommand(2.5)
      ),
      new ShooterStop(Shooter),
      new IndexerStop(Indexer),
      //Drive back
      new DriveEncoders(driveTrain, -0.5, 4, true),
      //Shoot Note and Drive Out
      autoBackwardsShoot(),
      new ParallelCommandGroup(
        Rotator.setArmGoalCommand(Constants.ArmConstants.kHome),
        new DriveEncoders(driveTrain, 0.6, 5, false)
      )
    );
  }
}
