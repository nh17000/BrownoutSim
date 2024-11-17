package frc.robot.subsystems.AmpBar;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class AmpBarSim implements AmpBarIO {
  private final SingleJointedArmSim armSim = new SingleJointedArmSim(
    DCMotor.getNEO(1), // i think it's a neo
    50, // ?
    SingleJointedArmSim.estimateMOI(Units.inchesToMeters(24.33), 2.07),
    Units.inchesToMeters(24.33), // check onshape
    Units.degreesToRadians(135), 
    Units.degreesToRadians(-60), 
    false, 
    0);

  private final PIDController controller;

  private final Translation3d AMP_BAR_TRANSLATION_ON_ROBOT = new Translation3d(0.068, 0, 0.632);

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

    Pose3d ampBarPoseToRobot =
                new Pose3d(AMP_BAR_TRANSLATION_ON_ROBOT, new Rotation3d(0, armSim.getAngleRads(), 0));
    Logger.recordOutput("MechanismPoses/Amp Bar", new Pose3d[] {ampBarPoseToRobot});
  }
  
  @Override
  public void setReference(double reference) {
    armSim.setInputVoltage(MathUtil.clamp(
      controller.calculate(armSim.getAngleRads(), reference), -12, 12));
    controller.reset(); // ?
  }
}
