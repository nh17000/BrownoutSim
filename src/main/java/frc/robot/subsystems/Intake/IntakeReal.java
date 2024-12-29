package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.lib.drivers.PearadoxSparkMax;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.VisionConstants;

public class IntakeReal implements IntakeIO {
  private PearadoxSparkMax utbRoller;
  private Debouncer debouncer;

  private static final NetworkTable llTable = NetworkTableInstance.getDefault().getTable(VisionConstants.INTAKE_LL_NAME);

  public IntakeReal() {
    utbRoller = new PearadoxSparkMax(IntakeConstants.UTB_ROLLER_ID, MotorType.kBrushless, IdleMode.kCoast, 80, false);
    debouncer = new Debouncer(0.5, DebounceType.kRising);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeCurrent = utbRoller.getOutputCurrent();
    inputs.hasTarget = debouncer.calculate(llTable.getEntry("tv").getDouble(0) == 1);
  }

  @Override
  public void set(double speed) {
    utbRoller.set(speed);
  }
}
