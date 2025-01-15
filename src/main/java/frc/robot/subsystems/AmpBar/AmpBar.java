// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AmpBar;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.SmarterDashboard;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.AmpBarConstants;

public class AmpBar extends SubsystemBase {
  private double ampBarAdjust = 0;

  public enum AmpBarMode{
    Stowed, Deployed, Trap, Climb, Defense
  }

  public AmpBarMode ampBarMode = AmpBarMode.Stowed;

  private static final AmpBar AMP_BAR = 
    new AmpBar(Robot.isReal() ? new AmpBarReal() : new AmpBarSim());

  public static AmpBar getInstance(){
    return AMP_BAR;
  }

  private AmpBarIO io;
  private final AmpBarIOInputsAutoLogged inputs = new AmpBarIOInputsAutoLogged();

  /** Creates a new AmpBar. */
  public AmpBar(AmpBarIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    if(RobotContainer.opController.getPOV() == 90){
      ampBarAdjust += 0.06;
    }
    else if(RobotContainer.opController.getPOV() == 270){
      ampBarAdjust -= 0.06;
    }

    io.updateInputs(inputs);
    Logger.processInputs("Amp Bar", inputs);

    SmarterDashboard.putString("Amp Bar Mode", ampBarMode.toString(), "Amp Bar");
  }

  public void setStowedMode(){
    ampBarMode = AmpBarMode.Stowed;
  }

  public void setDeployedMode(){
    ampBarMode = AmpBarMode.Deployed;
  }

  public void setTrapMode(){
    ampBarMode = AmpBarMode.Trap;
  }

  public void setClimbMode(){
    ampBarMode = AmpBarMode.Climb;
  }

  public void setDefenseMode(){
    ampBarMode = AmpBarMode.Defense;
  }

  public AmpBarMode getAmpBarMode(){
    return ampBarMode;
  }

  public void ampBarHold(){
    AmpBarMode ampBarMode = getAmpBarMode();
    if(ampBarMode == AmpBarMode.Deployed){
      io.setReference(
        AmpBarConstants.DEPLOYED_ROT + ampBarAdjust);
    }
    else if(ampBarMode == AmpBarMode.Climb){
      io.setReference(
        AmpBarConstants.CLIMB_ROT + ampBarAdjust);
    }
    else if(ampBarMode == AmpBarMode.Trap){
      io.setReference(
        AmpBarConstants.TRAP_ROT + ampBarAdjust);
    }
    else if(ampBarMode == AmpBarMode.Defense){
      io.setReference(
        AmpBarConstants.DEFENSE_ROT + ampBarAdjust);
    }
    else{
      io.setReference(
        AmpBarConstants.STOWED_ROT);
    }
  }
}
