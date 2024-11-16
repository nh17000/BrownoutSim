package frc.robot.subsystems.Drive.Module;

import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  class ModuleIOInputs {
    public double driveCurrent = 0.0;
    public double drivePosition = 0.0;
    public double driveVelocity = 0.0;    
    
    public double turnCurrent = 0.0;
    public double turnPosition = 0.0;
    public double turnVelocity = 0.0;

    public double absoluteWheelAngleDeg = 0.0;
    public double turnAngle = 0.0;
  }

  public default void updateInputs(ModuleIOInputs inputs) {}
  
  public default void setDriveBrake(boolean brake) {}

  public default void resetEncoders() {}

  public default void setSpeed(double speed) {}

  public default void setAngle(double currPos, double angleRadians) {}

  public default void stop() {}

  public int getDeviceID();
}
