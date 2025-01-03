// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
// import frc.robot.subsystems.Climber;
import frc.robot.subsystems.AmpBar.AmpBar;
import frc.robot.subsystems.AmpBar.AmpBar.AmpBarMode;

public class AmpBarHold extends Command {
  AmpBar ampBar = AmpBar.getInstance();
  // Climber climber = Climber.getInstance();

  /** Creates a new AmpBarHold. */
  public AmpBarHold() {
    addRequirements(ampBar);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ampBar.ampBarHold();

    if(RobotContainer.driverController.getRawButton(XboxController.Button.kA.value)){
      ampBar.setDefenseMode();
    }
    else if(RobotContainer.driverController.getLeftTriggerAxis() >= 0.95 && (ampBar.getAmpBarMode() == AmpBarMode.Stowed || ampBar.getAmpBarMode() == AmpBarMode.Defense)){
      ampBar.setDeployedMode();
    }
    else if (RobotContainer.driverController.getLeftTriggerAxis() < 0.95 && ampBar.getAmpBarMode() == AmpBarMode.Deployed){
      ampBar.setStowedMode();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
