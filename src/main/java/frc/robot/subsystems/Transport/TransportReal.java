package frc.robot.subsystems.Transport;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.drivers.PearadoxSparkFlex;
import frc.robot.Constants.TransportConstants;

public class TransportReal implements TransportIO {
  private PearadoxSparkFlex transportMotor;

  private DigitalInput irSensor;
  private Debouncer debouncer;

  public TransportReal() {
    transportMotor = new PearadoxSparkFlex(TransportConstants.TRANSPORT_ID, MotorType.kBrushless, IdleMode.kBrake, 60, false);

    irSensor = new DigitalInput(TransportConstants.IR_SENSOR_CHANNEL);
    debouncer = new Debouncer(0.2, DebounceType.kFalling);
  }

  @Override
  public void updateInputs(TransportIOInputs inputs) {
    inputs.hasNote = debouncer.calculate(!irSensor.get());
    inputs.transportCurrent = transportMotor.getOutputCurrent();
  }

  @Override
  public void set(double speed) {
    transportMotor.set(speed);
  }

  @Override
  public void setBrakeMode(boolean brake) {
    transportMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
