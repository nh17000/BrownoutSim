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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.RobotContainer;

public class ShooterSim implements ShooterIO {
  private double angle = Units.degreesToRadians(-85), intended = angle,
                 pivotVolts = 0.0, flywheelVolts = 0.0;

  private final SingleJointedArmSim pivotSim = new SingleJointedArmSim(
      DCMotor.getNEO(1),
      125.0,
      SingleJointedArmSim.estimateMOI(Units.inchesToMeters(16), 9),
      Units.inchesToMeters(16),
      Units.degreesToRadians(-90), 
      Units.degreesToRadians(0), 
      false, 
      angle);

  // private final FlywheelSim flywheelSim;

  private final PIDController pivotController;

  private final AbstractDriveTrainSimulation driveSim; 

  public ShooterSim(AbstractDriveTrainSimulation driveSim) {
    pivotSim.setState(Units.degreesToRadians(-85), 0);
    pivotController = new PIDController(
      ShooterConstants.PIVOT_SIM_kP, 0.0, 0.0);
    
    this.driveSim = driveSim;
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    pivotSim.update(0.02);
    // if (pivotSim.hasHitUpperLimit() || pivotSim.hasHitLowerLimit()) System.err.println("womp womp");
    this.angle = pivotSim.getAngleRads();

    inputs.shooterPivotCurrent = pivotSim.getCurrentDrawAmps();
    inputs.shooterPivotPos = this.angle;
    inputs.shooterPivotIntendedPos = this.intended;
    inputs.shooterPivotVolts = this.pivotVolts;

    Logger.recordOutput("MechanismPoses/Shooter", new Pose3d(
      ShooterConstants.SHOOTER_TRANSLATION_ON_ROBOT, new Rotation3d(0, this.angle, 0)));

    Logger.recordOutput("MechanismPoses/Shooter Intended", new Pose3d(
      ShooterConstants.SHOOTER_TRANSLATION_ON_ROBOT, new Rotation3d(0, this.intended, 0)));
  }

  @Override
  public void setPivotReference(double reference) {
    // angle -> encoder value -> angle
    intended = Units.degreesToRadians(Shooter.getInstance().pivotLerp.inverseInterpolate(reference) - 90);

    pivotVolts = MathUtil.clamp(
      pivotController.calculate(pivotSim.getAngleRads(), intended), -12, 12);

    pivotSim.setInputVoltage(pivotVolts);
  }

  @Override
  public double calculatePivotAngle(boolean isRedAlliance) {
    double x, z;

    // TODO: vision sim
    // if(hasPriorityTarget(isRedAlliance)){
    //   botpose_targetspace = llTable.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
      
    //   x = Math.abs(botpose_targetspace[0]);
    //   z = Math.abs(botpose_targetspace[2]) + 0.07;
    // }
    // else{

    int tagID = isRedAlliance ? 4 : 7;
    Pose2d tagPose = RobotContainer.aprilTagFieldLayout.getTagPose(tagID).get().toPose2d();

    boolean usePoseEstimate = false; // testing
    if (!usePoseEstimate) { RobotContainer.poseEstimation.resetPose(driveSim.getSimulatedDriveTrainPose()); }
    Pose2d robotPose = usePoseEstimate ? RobotContainer.poseEstimation.getEstimatedPose() : driveSim.getSimulatedDriveTrainPose();

    z = tagPose.getX() - robotPose.getX() + 0.07;
    x = (tagPose.getY() - 0.11) - robotPose.getY();
    // }

    double hypot = Math.hypot(x, z);

    double angle = Math.atan((FieldConstants.SPEAKER_HEIGHT - ShooterConstants.FLOOR_TO_SHOOTER) / hypot);
    return Units.radiansToDegrees(angle);
  }

  @Override
  public boolean hasPriorityTarget(boolean isRedAlliance) {
    // TODO: vision sim
    return false;
  }

  @Override
  public void setShooterVolts(double leftVolts, double rightVolts) {
    flywheelVolts = leftVolts;
  }

  @Override
  public void shootNote() {
    Pose2d robotSimulationWorldPose = driveSim.getSimulatedDriveTrainPose();
    ChassisSpeeds chassisSpeedsFieldRelative = driveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative();

    // TODO: test...why isn't this running?
    SimulatedArena.getInstance()
        .addGamePieceProjectile(
            new NoteOnFly(
                    robotSimulationWorldPose
                        .getTranslation(), // specify the position of the chassis
                    ShooterConstants.SHOOTER_TRANSLATION_ON_ROBOT.toTranslation2d(), // the shooter is installed at this position on the robot (in reference
                    // to the robot chassis center)
                    chassisSpeedsFieldRelative, // specify the field-relative speed of the chassis
                    // to add it to the initial velocity of the projectile
                    robotSimulationWorldPose
                        .getRotation(), // the shooter facing is the robot's facing
                    0.45, // initial height of the flying note
                    flywheelVolts / ShooterConstants.SPEAKER_VOLTAGE * 20, // we [5516] think the launching speed is proportional to the rpm, and is 16
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
