// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AmpBar;

import org.littletonrobotics.junction.Logger;

// import com.revrobotics.CANSparkBase.ControlType;
// import com.revrobotics.CANSparkBase.IdleMode;
// import com.revrobotics.CANSparkLowLevel.MotorType;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.lib.drivers.PearadoxSparkMax;
import frc.lib.util.SmarterDashboard;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.AmpBarConstants;

public class AmpBar extends SubsystemBase {
  // private PearadoxSparkMax ampBar;

  // private RelativeEncoder ampBarEncoder;
  // private SparkPIDController ampBarController;

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
    // ampBar = new PearadoxSparkMax(AmpBarConstants.AMP_BAR_ID, MotorType.kBrushless, IdleMode.kBrake, 40, false, 
    //   AmpBarConstants.AMP_BAR_kP, AmpBarConstants.AMP_BAR_kI, AmpBarConstants.AMP_BAR_kD, 
    //   AmpBarConstants.AMP_BAR_MIN_OUTPUT, AmpBarConstants.AMP_BAR_MAX_OUTPUT);

    // ampBarEncoder = ampBar.getEncoder();
    // ampBarController = ampBar.getPIDController();
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

    // SmarterDashboard.putNumber("Amp Bar Position", ampBarEncoder.getPosition(), "Amp Bar");
    // SmarterDashboard.putNumber("Amp Bar Adjust", ampBarAdjust, "Amp Bar");
    // SmarterDashboard.putNumber("Amp Bar Current", ampBar.getOutputCurrent(), "Amp Bar");
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
