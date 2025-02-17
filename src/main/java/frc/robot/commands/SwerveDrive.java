// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drive.Drivetrain;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.Shooter.ShooterMode;

public class SwerveDrive extends Command {
  private Drivetrain drivetrain = Drivetrain.getInstance();
  private Shooter shooter = Shooter.getInstance();
  private XboxController driverController = RobotContainer.driverController;
  

  /** Creates a new SwerveDrive. */
  public SwerveDrive() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(drivetrain.getDriveMode() == Drivetrain.DriveMode.Align){
      if(shooter.getShooterMode() == ShooterMode.SourcePassing){
        if(drivetrain.isRedAlliance()){
          drivetrain.swerveDrive(
            -driverController.getLeftY(), 
            -driverController.getLeftX(), 
            -1,
            1,
            true,
            new Translation2d(),
            true,
            true,
            true);
        }
        else{
          drivetrain.swerveDrive(
            -driverController.getLeftY(), 
            -driverController.getLeftX(), 
            0.5,
            0.866,
            true,
            new Translation2d(),
            true,
            true,
            true);
        }
      }
      else if(shooter.getShooterMode() == ShooterMode.AmpPassing){
        drivetrain.swerveDrive(
            -driverController.getLeftY(), 
            -driverController.getLeftX(), 
            0,
            1,
            true,
            new Translation2d(),
            true,
            true,
            true);
      }
      else{
        drivetrain.swerveDrive(
          -driverController.getLeftY(), 
          -driverController.getLeftX(), 
          -drivetrain.getAlignSpeed(),
          true,
          new Translation2d(),
          true);
      }

      if(drivetrain.readyToShoot() && shooter.readyToShoot()){
        CommandScheduler.getInstance().schedule(drivetrain.rumbleController());
      }
    }
    else if(drivetrain.getDriveMode() == Drivetrain.DriveMode.NoteAlign){
      drivetrain.swerveDrive(
          0.5, 
          0, 
          -drivetrain.getNoteAlignSpeed(),
          false,
          new Translation2d(),
          true);
    }
    else{
      drivetrain.swerveDrive(
        -driverController.getLeftY(), 
        -driverController.getLeftX(), 
        -driverController.getRightX(),
        RobotContainer.driverController.getRightTriggerAxis() < 0.9,
        new Translation2d(),
        true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
