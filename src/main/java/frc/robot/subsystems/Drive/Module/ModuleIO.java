package frc.robot.subsystems.Drive.Module;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    public double driveCurrent = 0.0;
    public double drivePosition = 0.0;
    public double driveVelocity = 0.0;    
    public double driveVolts = 0.0;    
    
    public double turnCurrent = 0.0;
    public double turnPosition = 0.0;
    public double turnVelocity = 0.0;
    public double turnVolts = 0.0;

    public double absoluteWheelAngleDeg = 0.0;
    public double turnAngle = 0.0;
    public double offset = 0.0;
  }

  public default void updateInputs(ModuleIOInputs inputs) {}
  
  public default void setDriveBrake(boolean brake) {}

  public default void resetEncoders() {}

  public default void setVelocity(double speed) {}

  public default void setAngle(Rotation2d angle) {}

  public default void setDriveOpenLoop(double output) {}

  public default void setTurnOpenLoop(double output) {}

  public int getDeviceID();
}
