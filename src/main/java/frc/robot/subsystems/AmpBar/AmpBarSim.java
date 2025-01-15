package frc.robot.subsystems.AmpBar;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.util.LerpTable;
import frc.robot.Constants.AmpBarConstants;

public class AmpBarSim implements AmpBarIO {
  private double angle = 0.0, volts = 0.0, intendedAngle = angle;

  private final SingleJointedArmSim armSim = new SingleJointedArmSim(
    DCMotor.getNEO(1),
    25, // 25:1 gearbox connected to a 15-30 tooth sprocket
    SingleJointedArmSim.estimateMOI(Units.inchesToMeters(24.33), 2.07),
    Units.inchesToMeters(24.33),
    Units.degreesToRadians(-90),
    Units.degreesToRadians(135),
    false, 
    angle);

  private LerpTable posToRadLerp = new LerpTable();

  private final PIDController controller;


  public AmpBarSim() {
    armSim.setState(angle, 0.0);

    controller = new PIDController(
      AmpBarConstants.AMP_BAR_SIM_kP, 0.0, 0.0);     

    posToRadLerp.addPoint(AmpBarConstants.STOWED_ROT,   Units.degreesToRadians(130));
    posToRadLerp.addPoint(AmpBarConstants.DEFENSE_ROT,  Units.degreesToRadians(0));
    posToRadLerp.addPoint(AmpBarConstants.DEPLOYED_ROT, Units.degreesToRadians(-60));
  }
  
  @Override
  public void updateInputs(AmpBarIOInputs inputs) {
    armSim.update(0.02);
    // if (armSim.hasHitUpperLimit() || armSim.hasHitLowerLimit()) System.err.println("uh oh");
    this.angle = armSim.getAngleRads();

    inputs.ampBarCurrent = armSim.getCurrentDrawAmps();
    inputs.ampBarPos = this.angle; 
    inputs.ampBarIntendedPos = this.intendedAngle;
    inputs.ampBarVolts = this.volts;

    Logger.recordOutput("MechanismPoses/Amp Bar", new Transform3d(
      AmpBarConstants.AMP_BAR_TRANSLATION_ON_ROBOT, new Rotation3d(0, this.angle, 0)));

    Logger.recordOutput("MechanismPoses/Amp Bar Intended", new Transform3d(
      AmpBarConstants.AMP_BAR_TRANSLATION_ON_ROBOT, new Rotation3d(0, this.intendedAngle, 0)));
  }
  
  @Override
  public void setReference(double reference) {
    intendedAngle = posToRadLerp.interpolate(reference);

    volts = MathUtil.clamp(
      controller.calculate(armSim.getAngleRads(), intendedAngle),
      -12, 12);

    armSim.setInputVoltage(volts);
  }
}
