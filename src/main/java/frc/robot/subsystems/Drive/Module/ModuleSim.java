package frc.robot.subsystems.Drive.Module;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

import edu.wpi.first.math.controller.PIDController;;

public class ModuleSim implements ModuleIO {
  private final SwerveModuleSimulation moduleSim;
  private final int index;

  private final PIDController turnFeedback;

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
    inputs.turnAngle = (inputs.absoluteWheelAngleDeg) * 2 * Math.PI;
  }

  @Override
  public int getDeviceID() {
    return index + 1;
  }

  @Override
  public void setSpeed(double speed) {
    // TODO: battery sim
    moduleSim.requestDriveVoltageOut(speed * 12);
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
