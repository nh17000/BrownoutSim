package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.drivers.PearadoxSparkFlex;
import frc.lib.drivers.PearadoxSparkMax;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.TransportConstants;
import frc.robot.Constants.VisionConstants;

public class IntakeReal implements IntakeIO {
  private PearadoxSparkMax utbRoller;
  private Debouncer noteDetectionDebouncer;

  private PearadoxSparkFlex transportMotor;

  private DigitalInput irSensor;
  private Debouncer irDebouncer;  

  private static final NetworkTable llTable = NetworkTableInstance.getDefault().getTable(VisionConstants.INTAKE_LL_NAME);

  public IntakeReal() {
    utbRoller = new PearadoxSparkMax(IntakeConstants.UTB_ROLLER_ID, MotorType.kBrushless, IdleMode.kCoast, 80, false);
    noteDetectionDebouncer = new Debouncer(0.5, DebounceType.kRising);

    transportMotor = new PearadoxSparkFlex(TransportConstants.TRANSPORT_ID, MotorType.kBrushless, IdleMode.kBrake, 60, false);
    irSensor = new DigitalInput(TransportConstants.IR_SENSOR_CHANNEL);
    irDebouncer = new Debouncer(0.2, DebounceType.kFalling);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeCurrent = utbRoller.getOutputCurrent();
    inputs.hasTarget = noteDetectionDebouncer.calculate(llTable.getEntry("tv").getDouble(0) == 1);
    inputs.intakeVolts = utbRoller.getAppliedOutput();
    
    inputs.hasNote = irDebouncer.calculate(!irSensor.get());
    inputs.transportCurrent = transportMotor.getOutputCurrent();
    inputs.transportVolts = transportMotor.getAppliedOutput();
  }

  @Override
  public void setIntake(double speed) {
    utbRoller.set(speed);
  }

  @Override
  public void setTransport(double speed) {
    transportMotor.set(speed);
  }

  @Override
  public void setTransportBrakeMode(boolean brake) {
    transportMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
