package frc.robot.subsystems.AmpBar;

import org.littletonrobotics.junction.AutoLog;

public interface AmpBarIO {
  @AutoLog
  public static class AmpBarIOInputs {
    public double ampBarPos = 0.0;
    public double ampBarCurrent = 0.0;
  }

  public default void updateInputs(AmpBarIOInputs inputs) {}

  public default void setReference(double reference) {}
}
