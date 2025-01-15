package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Radians;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.crescendo2024.NoteOnFly;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
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

    inputs.leftShooterVolts = this.flywheelVolts + ShooterConstants.LEFT_TO_RIGHT_VOLTAGE_OFFSET;
    inputs.rightShooterVolts = this.flywheelVolts;

    Logger.recordOutput("MechanismPoses/Shooter", getShooterTransform(this.angle));
    Logger.recordOutput("MechanismPoses/Shooter Intended", getShooterTransform(this.intended));
  }

  @Override
  public void setPivotReference(double reference) {
    // converts encoder value back to an angle
    this.intended = Units.degreesToRadians(
      Shooter.getInstance().pivotLerp.inverseInterpolate(reference) 
      - 90 // shooter starts vertically, so -90 is horizontal
      + 5 // adds 5 degrees to compensate for sim launch velocity -- maple sim uses g=11?
    );

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

    // currently, the odometry simulation is inaccurate, because the sim PID isn't tuned well
    // so it resets its odometry every tick, as if its odometry is perfect
    // set this boolean to false to model simulated odometry as it would behave on a real robot
    boolean usePoseEstimate = false;
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
    flywheelVolts = rightVolts;
  }

  public Transform3d getShooterTransform() {
    angle = pivotSim.getAngleRads();
    return getShooterTransform(angle);
  }

  public Transform3d getShooterTransform(double pitch) {
    return new Transform3d(
      ShooterConstants.SHOOTER_TRANSLATION_ON_ROBOT, new Rotation3d(0, pitch, 0));
  }

  // position = percentage of the way from the intake to the flywheels, 0 to 1, default 0.5
  public Transform3d getNoteInShooterTransform(Transform3d shooterPose, double position) {
    Transform3d noteToShooter = new Transform3d(
      new Translation3d(
        -ShooterConstants.SHOOTER_TRANSLATION_ON_ROBOT.getX() + 0.148, 
        -ShooterConstants.SHOOTER_TRANSLATION_ON_ROBOT.getY(), 
        -ShooterConstants.SHOOTER_TRANSLATION_ON_ROBOT.getZ() + 0.04 + position * 0.36),
      new Rotation3d(0, Math.PI / 2.0, 0));
    
    return shooterPose.plus(noteToShooter);
  }

  @Override
  public void visualizeHeldNote(double pos) {
    // TODO: create a compressed note CAD (current one portrudes through sides of shooter)
    
    // visualizing the held note as a component of the robot is more stable
    // as the robot pose would otherwise one tick behind when this method is called
    // however, this means having to hide the note component below the field
    boolean visualizeAsComponent = true;
    if (visualizeAsComponent) {
      // visualizes the held note as a component of the robot in a Transform3d
      // in advantagescope, add FieldSimulation/Held Note as a component to Brownout
      if (pos >= 0) { // position is 0-1, indicating how far it is through the intake/shooter
        Logger.recordOutput("MechanismPoses/Held Note", 
          getNoteInShooterTransform(getShooterTransform(), pos));
      } else {
        // the note component is hidden 5 meters below the robot
        Logger.recordOutput("MechanismPoses/Held Note", 
          new Transform3d(0.0, 0.0, -5.0, new Rotation3d()));
      }      
    } else { 
      // visualizes the held note as a unique game piece
      // in advantagescope, add FieldSimulation/Held Note as a Note game piece
      Pose3d robotPose = new Pose3d(driveSim.getSimulatedDriveTrainPose());
      if (pos >= 0) { // position is 0-1, indicating how far it is through the intake/shooter
        Logger.recordOutput("FieldSimulation/Held Note", 
          robotPose.transformBy(getNoteInShooterTransform(getShooterTransform(), pos)));
      } else {
        // the note component is hidden 5 meters below the robot
        Logger.recordOutput("FieldSimulation/Held Note", 
          new Pose3d(0.0, 0.0, -5.0, new Rotation3d()));
      }
    }
    
    // TODO: climber sim
    // currently visualizes the climbers as components at their default zero
    Logger.recordOutput("MechanismPoses/Left Climber", Transform3d.kZero);
    Logger.recordOutput("MechanismPoses/Right Climber", Transform3d.kZero);
  }

  // visualizes a note game piece as a projectile launched from the shooter
  // at the angle of its pivot and a velocity proportional to the voltage of the flywheels
  // TODO: visualize amp shots at deflecting off the amp bar
  @Override
  public void shootNote() {
    Pose2d robotSimulationWorldPose = driveSim.getSimulatedDriveTrainPose();
    ChassisSpeeds chassisSpeedsFieldRelative = driveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative();
    Transform3d shooterToRobot = getShooterTransform();
    Transform3d stowedNoteTransform = getNoteInShooterTransform(shooterToRobot, 0.45);
    Transform3d launchNoteTransform = getNoteInShooterTransform(shooterToRobot, 1);

    SimulatedArena.getInstance()
        .addGamePieceProjectile(
            new NoteOnFly(
                    robotSimulationWorldPose.getTranslation(), // specify the position of the chassis
                    stowedNoteTransform.getTranslation().toTranslation2d(), // the shooter is installed at this position on the robot (in reference
                    // to the robot chassis center)
                    chassisSpeedsFieldRelative, // specify the field-relative speed of the chassis
                    // to add it to the initial velocity of the projectile
                    robotSimulationWorldPose.getRotation().rotateBy(new Rotation2d(Math.PI)), // shooter on the back of the robot
                    launchNoteTransform.getZ(), // initial height of the flying note
                    ShooterConstants.FLYWHEEL_VOLTAGE_TO_LAUNCH_VELOCITY * flywheelVolts, 
                    // we [5516] think the launching speed is proportional to the rpm, and is 16
                    // meters/second when the motor rpm is 6000
                    // TODO: flywheel sim
                    // the note is launched at the angle of the pivot
                    launchNoteTransform.getRotation().getMeasureY().in(Radians)
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
