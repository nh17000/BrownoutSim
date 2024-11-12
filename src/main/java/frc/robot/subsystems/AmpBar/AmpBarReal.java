package frc.robot.subsystems.AmpBar;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import frc.lib.drivers.PearadoxSparkMax;
import frc.robot.Constants.AmpBarConstants;

public class AmpBarReal implements AmpBarIO {
  private PearadoxSparkMax ampBar;
  private RelativeEncoder ampBarEncoder;
  private SparkPIDController ampBarController;

  public AmpBarReal() {
    ampBar = new PearadoxSparkMax(AmpBarConstants.AMP_BAR_ID, MotorType.kBrushless, IdleMode.kBrake, 40, false, 
      AmpBarConstants.AMP_BAR_kP, AmpBarConstants.AMP_BAR_kI, AmpBarConstants.AMP_BAR_kD, 
      AmpBarConstants.AMP_BAR_MIN_OUTPUT, AmpBarConstants.AMP_BAR_MAX_OUTPUT);

    ampBarEncoder = ampBar.getEncoder();
    ampBarController = ampBar.getPIDController();
  }

  @Override
  public void updateInputs(AmpBarIOInputs inputs) {
    inputs.ampBarPos = ampBarEncoder.getPosition();
    inputs.ampBarCurrent = ampBar.getOutputCurrent();
  }

  @Override
  public void setReference(double reference) {
    ampBarController.setReference(
        reference,
        ControlType.kPosition,
        0);
  }
}