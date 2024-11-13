package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  class ShooterIOInputs {
    public double shooterPivotPos = 0.0;
    public double shooterPivotCurrent = 0.0;

    public double leftShooterSpeed = 0.0;
    public double rightShooterSpeed = 0.0;

    public double leftShooterStatorCurrent = 0.0;
    public double rightShooterStatorCurrent = 0.0;
    public double leftShooterSupplyCurrent = 0.0;
    public double rightShooterSupplyCurrent = 0.0;
  }
  
  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setShooterVolts(double leftVolts, double rightVolts) {}

  public default void setShooterSpeed(double leftSpeed, double rightSpeed) {}

  public default void setPivotSpeed(double speed) {}

  public default void setPivotReference(double reference) {}

  public default void resetPivotEncoder() {}

  public default void setBrakeMode(boolean brake) {}

  public default void setCurrentLimit(double limit) {}

  public default void setPipeline(int index) {}

  public double calculatePivotAngle(boolean isRedAlliance);

  public boolean hasPriorityTarget(boolean isRedAlliance);

  public default void shootNote() {}
}
