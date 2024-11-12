package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ShooterSim implements ShooterIO {
  private final SingleJointedArmSim pivotSim = new SingleJointedArmSim(
      DCMotor.getKrakenX60(1), // i think it's a neo
      50, // ?
      0.5, // ??
      0.5, // check onshape
      Units.degreesToRadians(-85), 
      Units.degreesToRadians(0), 
      true, 
      0);

  private final PIDController pivotController;

  public ShooterSim() {
    pivotSim.setState(0, 0);
    pivotController = new PIDController(0, 0, 0);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    pivotSim.update(0.02);
    inputs.shooterPivotCurrent = pivotSim.getCurrentDrawAmps();
    // TODO: conv from encoder pos to radians
    inputs.shooterPivotPos = pivotSim.getAngleRads();
  }

  @Override
  public void setPivotReference(double reference) {
    pivotSim.setInputVoltage(MathUtil.clamp(
      pivotController.calculate(pivotSim.getAngleRads(), reference), -12, 12));
    pivotController.reset(); // ?
  }

  @Override
  public double calculatePivotAngle(boolean isRedAlliance) {
    // TODO: vision sim
    return 0;
  }

  @Override
  public boolean hasPriorityTarget(boolean isRedAlliance) {
    // TODO: vision sim
    return false;
  }
}
