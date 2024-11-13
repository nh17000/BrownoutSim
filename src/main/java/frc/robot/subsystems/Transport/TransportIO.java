package frc.robot.subsystems.Transport;

import org.littletonrobotics.junction.AutoLog;

public interface TransportIO {
  @AutoLog
  public class TransportIOInputs {
    public double transportCurrent = 0.0;
    public boolean hasNote = false; // whether a note is indexed in the robot
    // TODO: add more inputs
  }

  public default void updateInputs(TransportIOInputs inputs) {}

  public default void set(double speed) {}

  public default void setBrakeMode(boolean brake) {}
}
