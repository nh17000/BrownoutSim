package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  class IntakeIOInputs {
    public double intakeCurrent = 0.0;
    public boolean hasTarget = false;
    // TODO: add more inputs
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void set(double speed) {}
}
