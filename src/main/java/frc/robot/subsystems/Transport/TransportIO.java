package frc.robot.subsystems.Transport;

import org.littletonrobotics.junction.AutoLog;

public interface TransportIO {
  @AutoLog
  public static class TransportIOInputs {
    public double transportCurrent = 0.0;
    public boolean hasNote = false;
  }

  public default void updateInputs(TransportIOInputs inputs) {}

  public default void set(double speed) {}

  public default void setBrakeMode(boolean brake) {}
}
