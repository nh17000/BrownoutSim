package frc.robot.subsystems.Shooter;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.crescendo2024.NoteOnFly;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.ShooterConstants;

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
  // private final FlywheelSim flywheelSim;

  private final PIDController pivotController;

  private final AbstractDriveTrainSimulation driveSim; 

  private final Translation3d SHOOTER_TRANSLATION_ON_ROBOT = new Translation3d(-0.108, 0, -0.154);  

  private double volts;

  public ShooterSim(AbstractDriveTrainSimulation driveSim) {
    pivotSim.setState(-85, 0);
    pivotController = new PIDController(0, 0, 0);
    
    this.driveSim = driveSim;
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    pivotSim.update(0.02);
    inputs.shooterPivotCurrent = pivotSim.getCurrentDrawAmps();
    // TODO: conv from encoder pos to radians
    inputs.shooterPivotPos = pivotSim.getAngleRads();

    Pose3d shooterPoseToRobot =
                new Pose3d(SHOOTER_TRANSLATION_ON_ROBOT, new Rotation3d(0, pivotSim.getAngleRads(), 0));
    Logger.recordOutput("MechanismPoses/Shooter", new Pose3d[] {shooterPoseToRobot});
  }

  @Override
  public void setPivotReference(double reference) {
    reference = 0; // test
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

  @Override
  public void setShooterVolts(double leftVolts, double rightVolts) {
    volts = leftVolts;
  }

  @Override
  public void shootNote() {
    Pose2d robotSimulationWorldPose = driveSim.getSimulatedDriveTrainPose();
    ChassisSpeeds chassisSpeedsFieldRelative = driveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative();

    SimulatedArena.getInstance()
        .addGamePieceProjectile(
            new NoteOnFly(
                    robotSimulationWorldPose
                        .getTranslation(), // specify the position of the chassis
                    SHOOTER_TRANSLATION_ON_ROBOT.toTranslation2d(), // the shooter is installed at this position on the robot (in reference
                    // to the robot chassis center)
                    chassisSpeedsFieldRelative, // specify the field-relative speed of the chassis
                    // to add it to the initial velocity of the projectile
                    robotSimulationWorldPose
                        .getRotation(), // the shooter facing is the robot's facing
                    0.45, // initial height of the flying note
                    volts / ShooterConstants.SPEAKER_VOLTAGE * 20, // we [5516] think the launching speed is proportional to the rpm, and is 16
                    // meters/second when the motor rpm is 6000
                    // i haven't implemented flywheelsim yet, so i'm assuming our speaker shots are 20 m/s
                    pivotSim.getAngleRads() // the note is launched at the angle of the pivot
                    )
                .asSpeakerShotNote(() -> System.out.println("hit target!!!"))
                .enableBecomeNoteOnFieldAfterTouchGround()
                .withProjectileTrajectoryDisplayCallBack(
                    (pose3ds) ->
                        Logger.recordOutput(
                            "Flywheel/NoteProjectileSuccessful", pose3ds.toArray(Pose3d[]::new)),
                    (pose3ds) ->
                        Logger.recordOutput(
                            "Flywheel/NoteProjectileUnsuccessful",
                            pose3ds.toArray(Pose3d[]::new))));
  }
}
