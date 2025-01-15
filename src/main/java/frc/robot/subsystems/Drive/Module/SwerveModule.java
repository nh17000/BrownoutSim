// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive.Module;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule extends SubsystemBase {
  private Rotation2d lastAngle;

  private ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

  /** Creates a new SwerveModule. */
  public SwerveModule(ModuleIO io) {
      this.io = io;
      lastAngle = getState().angle;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + Integer.toString(getDeviceID()), inputs);
  }

  public void setBrake(boolean brake){
    io.setDriveBrake(brake);
  }
  
  public double getDriveMotorPosition(){
    return inputs.drivePosition;
  }

  public double getDriveMotorVelocity(){
    return inputs.driveVelocity;
  }

  public double getTurnMotorPosition(){
    return inputs.turnPosition;
  }

  public double getTurnMotorVelocity(){
    return inputs.turnVelocity;
  }

  public double getAbsoluteEncoderAngle(){
    return inputs.turnAngle;
  }

  public void resetEncoders(){
    io.resetEncoders();
  }

  public SwerveModuleState getState(){
    return new SwerveModuleState(getDriveMotorVelocity(), new Rotation2d(getTurnMotorPosition()));
  }

  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(getDriveMotorPosition(), new Rotation2d(getTurnMotorPosition()));
  }

  public void setDesiredState(SwerveModuleState desiredState){
    desiredState.optimize(getState().angle); 
    
    setAngle(desiredState);
    setSpeed(desiredState);
  }

  public void setSpeed(SwerveModuleState desiredState){
    io.setVelocity(desiredState.speedMetersPerSecond / (SwerveConstants.WHEEL_DIAMETER / 2.0));
  }

  public void setAngle(SwerveModuleState desiredState){
    Rotation2d angle = 
      (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConstants.DRIVETRAIN_MAX_SPEED * 0.01)) 
      ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
    
    // io.setAngle(getTurnMotorPosition(), desiredState.angle.getRadians());
    // io.setAngle(getTurnMotorPosition(), angle.getRadians());
    io.setAngle(angle);

    lastAngle = angle;
  }

  public void stop(){
    io.setDriveOpenLoop(0.0);
    io.setTurnOpenLoop(0.0);
  }

  public int getDeviceID(){
    return io.getDeviceID();
  }
}
