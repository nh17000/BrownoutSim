package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double intakeCurrent = 0.0;
    public double transportCurrent = 0.0;

    public boolean hasNote = false;
    public boolean hasTarget = false;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setIntake(double speed) {}

  public default void setTransport(double speed) {}

  public default void setTransportBrakeMode(boolean brake) {}

  public default boolean obtainGamePieceFromIntake() { return false; }
}
