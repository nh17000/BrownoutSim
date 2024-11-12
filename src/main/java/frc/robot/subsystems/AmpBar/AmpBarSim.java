package frc.robot.subsystems.AmpBar;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class AmpBarSim implements AmpBarIO {
  private final SingleJointedArmSim armSim = new SingleJointedArmSim(
    DCMotor.getNEO(1), // i think it's a neo
    50, // ?
    0.5, // ??
    0.5, // check onshape
    Units.degreesToRadians(135), 
    Units.degreesToRadians(-60), 
    true, 
    0);

  private final PIDController controller;

  public AmpBarSim() {
    controller = new PIDController(0, 0, 0);
    armSim.setState(0, 0);
  }
  
  @Override
  public void updateInputs(AmpBarIOInputs inputs) {
    armSim.update(0.02);
    inputs.ampBarCurrent = armSim.getCurrentDrawAmps();
    // TODO: convert encoder values (roughly -20 to -1) to the actual angle of the arm (-135 to 60)
    inputs.ampBarPos = armSim.getAngleRads(); 
  }
  
  @Override
  public void setReference(double reference) {
    armSim.setInputVoltage(MathUtil.clamp(
      controller.calculate(armSim.getAngleRads(), reference), -12, 12));
    controller.reset(); // ?
  }
}
