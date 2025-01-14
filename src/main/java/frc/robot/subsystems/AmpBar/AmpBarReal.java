package frc.robot.subsystems.AmpBar;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.lib.drivers.PearadoxSparkMax;
import frc.robot.Constants.AmpBarConstants;

public class AmpBarReal implements AmpBarIO {
  private PearadoxSparkMax ampBar;
  private RelativeEncoder ampBarEncoder;
  private SparkClosedLoopController ampBarController;

  public AmpBarReal() {
    ampBar = new PearadoxSparkMax(AmpBarConstants.AMP_BAR_ID, MotorType.kBrushless, IdleMode.kBrake, 40, false, 
      AmpBarConstants.AMP_BAR_kP, AmpBarConstants.AMP_BAR_kI, AmpBarConstants.AMP_BAR_kD, 
      AmpBarConstants.AMP_BAR_MIN_OUTPUT, AmpBarConstants.AMP_BAR_MAX_OUTPUT);

    ampBarEncoder = ampBar.getEncoder();
    ampBarController = ampBar.getClosedLoopController();
  }

  @Override
  public void updateInputs(AmpBarIOInputs inputs) {
    inputs.ampBarPos = ampBarEncoder.getPosition();
    inputs.ampBarCurrent = ampBar.getOutputCurrent();
    inputs.ampBarVolts = ampBar.getAppliedOutput();
  }

  @Override
  public void setReference(double reference) {
    ampBarController.setReference(reference, ControlType.kPosition);
  }
}
