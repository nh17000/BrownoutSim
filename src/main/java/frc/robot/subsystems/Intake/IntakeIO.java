package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double intakeCurrent = 0.0;
    public boolean hasTarget = false;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void set(double speed) {}

  public default boolean obtainGamePieceFromIntake() { return false; }

  public default boolean simHasNote() { return false; }
}
