package frc.robot.subsystems.AmpBar;

import org.littletonrobotics.junction.AutoLog;

public interface AmpBarIO {
  @AutoLog
  class AmpBarIOInputs {
    public double ampBarPos = 0;
    public double ampBarCurrent = 0;
    // TODO: add more inputs
  }

  public default void updateInputs(AmpBarIOInputs inputs) {}

  public default void setReference(double reference) {}
}
