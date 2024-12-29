package frc.robot.subsystems.Drive.Module;

import static edu.wpi.first.units.Units.*;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.SwerveConstants;

public class ModuleSim implements ModuleIO {
  private final SwerveModuleSimulation moduleSim;
  private final int index;

  private boolean driveClosedLoop = false, turnClosedLoop = false;
  private final PIDController driveController = new PIDController(0.05, 0, 0);
  private final PIDController turnController = new PIDController(8.0, 0, 0);

  private double turnRelativeOffset = Double.MIN_VALUE;

  private double driveFFVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  private final SimulatedMotorController.GenericMotorController driveMotor;
  private final SimulatedMotorController.GenericMotorController turnMotor;

  public ModuleSim(SwerveModuleSimulation moduleSim, int index) {
    this.moduleSim = moduleSim;
    this.index = index;

    this.driveMotor = moduleSim
      .useGenericMotorControllerForDrive()
      .withCurrentLimit(Amps.of(SwerveConstants.DRIVE_CURRENT_LIMIT));
    this.turnMotor = moduleSim
      .useGenericControllerForSteer()
      .withCurrentLimit(Amps.of(SwerveConstants.TURN_CURRENT_LIMIT));

    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    if (driveClosedLoop) {
      driveAppliedVolts = driveFFVolts + driveController.calculate(
        moduleSim.getDriveWheelFinalSpeed().in(RadiansPerSecond));
    } else {
      driveController.reset();
    }

    if (turnClosedLoop) {
      turnAppliedVolts = turnController.calculate(
        moduleSim.getSteerAbsoluteFacing().getRadians());
    } else {
      turnController.reset();
    }

    driveMotor.requestVoltage(Volts.of(driveAppliedVolts));
    turnMotor.requestVoltage(Volts.of(turnAppliedVolts));

    inputs.driveCurrent = Math.abs(moduleSim.getDriveMotorSupplyCurrent().in(Amps));    
    inputs.drivePosition = moduleSim.getDriveWheelFinalPosition().in(Radians) 
      * SwerveConstants.DRIVE_MOTOR_PCONVERSION;
    inputs.driveVelocity = moduleSim.getDriveWheelFinalSpeed().in(RadiansPerSecond) 
      * SwerveConstants.DRIVE_MOTOR_VCONVERSION;
    inputs.driveVolts = driveAppliedVolts;
    
    inputs.turnCurrent = Math.abs(moduleSim.getSteerMotorSupplyCurrent().in(Amps));
    inputs.turnPosition = moduleSim.getSteerAbsoluteFacing().getRadians() 
      * SwerveConstants.TURN_MOTOR_PCONVERSION;
    inputs.turnVelocity = moduleSim.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond)
      * SwerveConstants.TURN_MOTOR_VCONVERSION;
    inputs.turnVolts = turnAppliedVolts;

    // TODO: fix possible offset inversion
    inputs.absoluteWheelAngleDeg = moduleSim.getSteerAbsoluteFacing().getDegrees();
    if (turnRelativeOffset == Double.MIN_VALUE && Math.abs(inputs.absoluteWheelAngleDeg) >= 1) {
      turnRelativeOffset = inputs.absoluteWheelAngleDeg + inputs.turnPosition;
    }

    inputs.offset = turnRelativeOffset;
    inputs.turnAngle = inputs.absoluteWheelAngleDeg - turnRelativeOffset;
  }

  @Override
  public int getDeviceID() {
    return index + 1;
  }

  @Override
  public void setVelocity(double velocity) {
    // driveMotor.requestVoltage(Volts.of(speed * 12));
    driveClosedLoop = true;
    driveFFVolts = SwerveConstants.SIM_DRIVE_KS * Math.signum(velocity)
                 + SwerveConstants.SIM_DRIVE_KV * velocity;
    driveController.setSetpoint(velocity);
  }

  @Override
  public void setAngle(Rotation2d angle) {
    // turnMotor.requestVoltage(Volts.of(turnFeedback.calculate(currPos, angleRadians)));
    turnClosedLoop = true;
    turnController.setSetpoint(angle.getRadians());
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveClosedLoop = false;
    driveAppliedVolts = output;
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnClosedLoop = false;
    turnAppliedVolts = output;
  }
}
