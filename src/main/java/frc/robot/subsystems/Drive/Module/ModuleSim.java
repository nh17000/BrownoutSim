package frc.robot.subsystems.Drive.Module;

import static edu.wpi.first.units.Units.*;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import edu.wpi.first.math.controller.PIDController;

public class ModuleSim implements ModuleIO {
  private final SwerveModuleSimulation moduleSim;
  private final int index;

  private final PIDController turnFeedback;
  private double turnRelativeOffset = Double.MIN_VALUE;

  private final SimulatedMotorController.GenericMotorController driveMotor;
  private final SimulatedMotorController.GenericMotorController turnMotor;

  public ModuleSim(SwerveModuleSimulation moduleSim, int index) {
    this.moduleSim = moduleSim;
    this.index = index;

    this.driveMotor = moduleSim
    .useGenericMotorControllerForDrive()
    .withCurrentLimit(Amps.of(120));
    this.turnMotor = moduleSim.useGenericControllerForSteer().withCurrentLimit(Amps.of(20));

    turnFeedback = new PIDController(7.0, 0.0, 0.0);
    turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.driveCurrent = moduleSim.getDriveMotorSupplyCurrent().magnitude();    
    inputs.drivePosition = moduleSim.getCachedDriveWheelFinalPositions()[0].magnitude();
    inputs.driveVelocity = moduleSim.getDriveWheelFinalSpeed().magnitude();
    
    inputs.turnCurrent = moduleSim.getSteerMotorSupplyCurrent().magnitude();
    inputs.turnPosition = moduleSim.getCachedSteerRelativeEncoderPositions()[0].magnitude();
    inputs.turnVelocity = moduleSim.getSteerRelativeEncoderVelocity().magnitude();

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
  public void setSpeed(double speed) {
    // TODO: drive PID
    driveMotor.requestVoltage(Volts.of(speed * 12));
  }

  @Override
  public void setAngle(double currPos, double angleRadians) {
    turnMotor.requestVoltage(Volts.of(turnFeedback.calculate(currPos, angleRadians)));
  }

  @Override
  public void stop() {
    driveMotor.requestVoltage(Volts.of(0.0));
    turnMotor.requestVoltage(Volts.of(0.0));
  }
}
