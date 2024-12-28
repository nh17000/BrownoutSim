package frc.robot.subsystems.Drive.Module;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

import edu.wpi.first.math.controller.PIDController;

public class ModuleSim implements ModuleIO {
  private final SwerveModuleSimulation moduleSim;
  private final int index;

  private final PIDController turnFeedback;
  private double turnRelativeOffset = Double.MIN_VALUE;

  public ModuleSim(SwerveModuleSimulation moduleSim, int index) {
    this.moduleSim = moduleSim;
    this.index = index;

    turnFeedback = new PIDController(7.0, 0.0, 0.0);
    turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.driveCurrent = moduleSim.getDriveMotorSupplyCurrentAmps();    
    inputs.drivePosition = moduleSim.getDriveWheelFinalPositionRad();
    inputs.driveVelocity = moduleSim.getDriveWheelFinalSpeedRadPerSec();
    
    inputs.turnCurrent = moduleSim.getSteerMotorSupplyCurrentAmps();
    inputs.turnPosition = moduleSim.getSteerRelativeEncoderPositionRad();
    inputs.turnVelocity = moduleSim.getSteerRelativeEncoderSpeedRadPerSec();

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
    // TODO: battery sim
    // if (index == 3) speed *= -1;
    moduleSim.requestDriveVoltageOut(speed * 5);
  }

  @Override
  public void setAngle(double currPos, double angleRadians) {
    moduleSim.requestSteerVoltageOut(turnFeedback.calculate(currPos, angleRadians));
  }

  @Override
  public void stop() {
    moduleSim.requestDriveVoltageOut(0.0);
    moduleSim.requestSteerVoltageOut(0.0);
  }
}
