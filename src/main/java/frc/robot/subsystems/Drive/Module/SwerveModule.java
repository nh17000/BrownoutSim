// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive.Module;

import org.littletonrobotics.junction.Logger;

// import com.ctre.phoenix6.BaseStatusSignal;
// import com.ctre.phoenix6.StatusSignal;
// import com.ctre.phoenix6.controls.DutyCycleOut;
// import com.ctre.phoenix6.hardware.CANcoder;
// import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.lib.drivers.PearadoxTalonFX;
// import frc.lib.util.SmarterDashboard;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule extends SubsystemBase {
  // private PearadoxTalonFX driveMotor;
  // private PearadoxTalonFX turnMotor;

  // private PIDController turnPIDController;
  // private CANcoder absoluteEncoder;

  // private double absoluteEncoderOffset;

  // private StatusSignal<Double> drivePosition;
  // private StatusSignal<Double> driveVelocity;
  // private StatusSignal<Double> turnPosition;
  // private StatusSignal<Double> turnVelocity;
  // private StatusSignal<Double> absoluteEncoderAngle;
  // private StatusSignal<Double> driveCurrent;
  // private StatusSignal<Double> turnCurrent;

  private Rotation2d lastAngle;

  // private DutyCycleOut driveMotorRequest = new DutyCycleOut(0.0);
  // private DutyCycleOut turnMotorRequest = new DutyCycleOut(0.0);

  private ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

  /** Creates a new SwerveModule. */
  public SwerveModule(ModuleIO io) {
      // this.absoluteEncoderOffset = absoluteEncoderOffset;

      // absoluteEncoder = new CANcoder(absoluteEncoderId);

      // driveMotor = new PearadoxTalonFX(driveMotorId, NeutralModeValue.Coast, 45, driveMotorReversed);
      // turnMotor = new PearadoxTalonFX(turnMotorId, NeutralModeValue.Brake, 25, turnMotorReversed);

      // turnPIDController = new PIDController(SwerveConstants.KP_TURNING, 0, 0);
      // turnPIDController.enableContinuousInput(-Math.PI, Math.PI);

      // drivePosition = driveMotor.getPosition();
      // driveVelocity = driveMotor.getVelocity();
      // turnPosition = turnMotor.getPosition();
      // turnVelocity = turnMotor.getVelocity();
      // absoluteEncoderAngle = absoluteEncoder.getAbsolutePosition();
      // driveCurrent = driveMotor.getStatorCurrent();
      // turnCurrent = turnMotor.getStatorCurrent();

      // BaseStatusSignal.setUpdateFrequencyForAll(50, drivePosition, driveVelocity, turnPosition, turnVelocity, absoluteEncoderAngle, driveCurrent, turnCurrent);
      // driveMotor.optimizeBusUtilization();
      // turnMotor.optimizeBusUtilization();

      // resetEncoders();
      this.io = io;
      lastAngle = getState().angle;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + Integer.toString(getDeviceID()), inputs);
    // This method will be called once per scheduler run
    // SmarterDashboard.putNumber("Drive Distance (rot) - Motor: " + driveMotor.getDeviceID(), getDriveMotorPosition(), "SwerveModule");
    // SmarterDashboard.putNumber("Wheel Position (rot) - Motor: " + driveMotor.getDeviceID(), getTurnMotorPosition(), "SwerveModule");
    // SmarterDashboard.putNumber("Drive Motor " + driveMotor.getDeviceID() + " Current", driveMotor.getStatorCurrent().getValueAsDouble(), "SwerveModule");
    // SmarterDashboard.putNumber("Turn Motor " + driveMotor.getDeviceID() + " Current", turnMotor.getStatorCurrent().getValueAsDouble(), "SwerveModule");
    // SmarterDashboard.putNumber("Absolute Wheel Angle (deg) - Motor: " + driveMotor.getDeviceID(), absoluteEncoder.getAbsolutePosition().getValueAsDouble(), "SwerveModule");
  }

  public void setBrake(boolean brake){
    // if(brake){
    //   driveMotor.setNeutralMode(NeutralModeValue.Brake);
    //   turnMotor.setNeutralMode(NeutralModeValue.Coast);
    // }
    // else{
    //   driveMotor.setNeutralMode(NeutralModeValue.Coast);
    //   turnMotor.setNeutralMode(NeutralModeValue.Coast);
    // }
    io.setDriveBrake(brake);
  }
  
  public double getDriveMotorPosition(){
    // return driveMotor.getPosition().getValueAsDouble() * SwerveConstants.DRIVE_MOTOR_PCONVERSION;
    return inputs.drivePosition;
  }

  public double getDriveMotorVelocity(){
    // return driveMotor.getVelocity().getValueAsDouble() * SwerveConstants.DRIVE_MOTOR_VCONVERSION;
    return inputs.driveVelocity;
  }

  public double getTurnMotorPosition(){
    // return turnMotor.getPosition().getValueAsDouble() * SwerveConstants.TURN_MOTOR_PCONVERSION;
    return inputs.turnPosition;
  }

  public double getTurnMotorVelocity(){
    // return turnMotor.getVelocity().getValueAsDouble() * SwerveConstants.TURN_MOTOR_VCONVERSION;
    return inputs.turnVelocity;
  }

  public double getAbsoluteEncoderAngle(){
    // double angle = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
    // angle -= absoluteEncoderOffset;
    // angle *= (2 * Math.PI);
    // return angle;
    return inputs.turnAngle;
  }

  public void resetEncoders(){
    // driveMotor.setPosition(0);
    // turnMotor.setPosition(getAbsoluteEncoderAngle() / SwerveConstants.TURN_MOTOR_PCONVERSION);
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
    // driveMotor.setControl(driveMotorRequest.withOutput(desiredState.speedMetersPerSecond / SwerveConstants.DRIVETRAIN_MAX_SPEED));
    io.setVelocity(desiredState.speedMetersPerSecond / (SwerveConstants.WHEEL_DIAMETER / 2.0));
  }

  public void setAngle(SwerveModuleState desiredState){
    Rotation2d angle = 
      (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConstants.DRIVETRAIN_MAX_SPEED * 0.01)) 
      ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
    
    // turnMotor.setControl(turnMotorRequest.withOutput(turnPIDController.calculate(getTurnMotorPosition(), desiredState.angle.getRadians())));

    // io.setAngle(getTurnMotorPosition(), desiredState.angle.getRadians());
    // io.setAngle(getTurnMotorPosition(), angle.getRadians());
    io.setAngle(angle);

    lastAngle = angle;
  }

  public void stop(){
    // driveMotor.setControl(driveMotorRequest.withOutput(0));
    // turnMotor.setControl(turnMotorRequest.withOutput(0));
    io.setDriveOpenLoop(0.0);
    io.setTurnOpenLoop(0.0);
  }

  public int getDeviceID(){
    return io.getDeviceID();
  }
}
