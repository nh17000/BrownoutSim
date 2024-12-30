// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;

// import com.revrobotics.CANSparkBase.IdleMode;
// import com.revrobotics.CANSparkLowLevel.MotorType;

// import edu.wpi.first.math.filter.Debouncer;
// import edu.wpi.first.math.filter.Debouncer.DebounceType;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.lib.drivers.PearadoxSparkMax;
// import frc.lib.util.SmarterDashboard;
// import frc.robot.Constants.IntakeConstants;
// import frc.robot.Constants.VisionConstants;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.TransportConstants;

public class Intake extends SubsystemBase {
  private boolean rumbled = false;
  private boolean transportIsHolding = true;

  private long shootTime = 0;

  private static Intake INTAKE;

  public static Intake createInstance(IntakeIO io) {
    if (INTAKE == null) {
      INTAKE = new Intake(io);
    } else {
      throw new IllegalStateException("Intake instance already created!");
    }
    return INTAKE;
  }

  public static Intake getInstance() {
    if (INTAKE == null) {
      throw new IllegalStateException("Intake instance not created");
    }
    return INTAKE;
  }
  
  private IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  /** Creates a new Intake. */
  private Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    if(transportIsHolding){
      if(hasNote()){
        transportStop();
      }
      else{
        transportHold();
      }
    }

    if(!rumbled && (inputs.intakeCurrent > IntakeConstants.RUMBLE_AT_CURRENT || hasNote())){
      CommandScheduler.getInstance().schedule(rumbleController());
      rumbled = true;
    }
    if(rumbled && !(inputs.intakeCurrent > IntakeConstants.RUMBLE_AT_CURRENT || hasNote())){
      rumbled = false;
    }
  }

  public void utbIntakeIn(){
    io.setIntake(IntakeConstants.INTAKE_IN_SPEED);
  }

  public void utbIntakeOut(){
    io.setIntake(IntakeConstants.INTAKE_OUT_SPEED);
  }

  public void utbIntakeStop(){
    io.setIntake(0);
  }

  public Command rumbleController(){
    return new InstantCommand(() -> RobotContainer.driverController.setRumble(RumbleType.kBothRumble, 0.75))
      .andThen(new InstantCommand(() -> RobotContainer.opController.setRumble(RumbleType.kBothRumble, 0.75)))
      .andThen(new WaitCommand(0.25))
      .andThen(new InstantCommand(() -> RobotContainer.driverController.setRumble(RumbleType.kBothRumble, 0)))
      .andThen(new InstantCommand(() -> RobotContainer.opController.setRumble(RumbleType.kBothRumble, 0)));
  }

  public boolean hasTarget(){
    return inputs.hasTarget;
  }
  
  public boolean hasNote(){
    return inputs.hasNote;
  }

  public void transportHold(){
    io.setTransport(TransportConstants.TRANSPORT_HOLD_SPEED);
  }

  public void transportOut(){
    io.setTransport(TransportConstants.TRANSPORT_OUT_SPEED);
  }

  public void transportStop(){
    io.setTransport(0);
  }

  public void transportShoot(){    
    shootTime = System.currentTimeMillis();
    io.setTransport(TransportConstants.TRANSPORT_SHOOT_SPEED);
  }

  public void setTransportBrakeMode(boolean brake){
    io.setTransportBrakeMode(brake);
  }

  public long getRequestedShootTime(){
    return shootTime;
  }

  public void setTransportIsHolding(boolean isHolding){
    this.transportIsHolding = isHolding;
  }
}
