package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.lib.drivers.PearadoxSparkFlex;
import frc.lib.drivers.PearadoxTalonFX;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotContainer;

public class ShooterReal implements ShooterIO {
  private PearadoxTalonFX leftShooter;
  private PearadoxTalonFX rightShooter;
  
  private PearadoxSparkFlex pivot;

  private RelativeEncoder pivotEncoder;

  private SparkClosedLoopController pivotController;

  private VoltageOut voltage_request = new VoltageOut(0);

  private static final NetworkTable llTable = NetworkTableInstance.getDefault().getTable(VisionConstants.SHOOTER_LL_NAME);
  private double[] botpose_targetspace = new double[6];

  public ShooterReal() {
    leftShooter = new PearadoxTalonFX(ShooterConstants.LEFT_SHOOTER_ID, NeutralModeValue.Coast, 50, true); 
    
    rightShooter = new PearadoxTalonFX(ShooterConstants.RIGHT_SHOOTER_ID, NeutralModeValue.Coast, 50, false);

    pivot = new PearadoxSparkFlex(ShooterConstants.PIVOT_ID, MotorType.kBrushless, IdleMode.kBrake, 50, true,
      ShooterConstants.PIVOT_kP, ShooterConstants.PIVOT_kI, ShooterConstants.PIVOT_kD,
      ShooterConstants.PIVOT_MIN_OUTPUT, ShooterConstants.PIVOT_MAX_OUTPUT);

    pivotEncoder = pivot.getEncoder();

    pivotController = pivot.getClosedLoopController();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.shooterPivotPos = pivotEncoder.getPosition();
    inputs.shooterPivotCurrent = pivot.getOutputCurrent();

    inputs.leftShooterSpeed = leftShooter.getVelocity().getValueAsDouble() * 60.0;
    inputs.rightShooterSpeed = rightShooter.getVelocity().getValueAsDouble() * 60.0;

    inputs.leftShooterVolts = leftShooter.getMotorVoltage().getValueAsDouble();
    inputs.rightShooterVolts = rightShooter.getMotorVoltage().getValueAsDouble();

    inputs.leftShooterStatorCurrent = leftShooter.getStatorCurrent().getValueAsDouble();
    inputs.rightShooterStatorCurrent = rightShooter.getStatorCurrent().getValueAsDouble();
    inputs.leftShooterSupplyCurrent = leftShooter.getSupplyCurrent().getValueAsDouble();
    inputs.rightShooterSupplyCurrent = rightShooter.getSupplyCurrent().getValueAsDouble();
  }
  
  @Override
  public void setShooterVolts(double leftVolts, double rightVolts) {
    leftShooter.setControl(voltage_request.withOutput(leftVolts));
    rightShooter.setControl(voltage_request.withOutput(rightVolts));    
  }
  
  @Override
  public void setShooterSpeed(double leftSpeed, double rightSpeed) {
    leftShooter.set(leftSpeed);
    rightShooter.set(rightSpeed);
  }

  @Override
  public void setPivotSpeed(double speed) {
    pivot.set(speed);
  }

  @Override
  public void setPivotReference(double reference) {
    pivotController.setReference(reference, ControlType.kPosition, 0);
  }

  @Override
  public void resetPivotEncoder() {
    pivotEncoder.setPosition(0);
  }

  @Override
  public void setBrakeMode(boolean brake) {
    if(brake){
      leftShooter.setNeutralMode(NeutralModeValue.Brake);
      rightShooter.setNeutralMode(NeutralModeValue.Brake);
      // pivot.setBrakeMode(IdleMode.kBrake);
    }
    else{
      leftShooter.setNeutralMode(NeutralModeValue.Coast);
      rightShooter.setNeutralMode(NeutralModeValue.Coast);
    }
  }

  @Override
  public void setCurrentLimit(double limit) {
    leftShooter.setCurrentLimit(limit);
    rightShooter.setCurrentLimit(limit);
  }

  @Override
  public double calculatePivotAngle(boolean isRedAlliance) {
    double x, z;

    if(hasPriorityTarget(isRedAlliance)){
      botpose_targetspace = llTable.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
      
      x = Math.abs(botpose_targetspace[0]);
      z = Math.abs(botpose_targetspace[2]) + 0.07;
    }
    else{
      int tagID = isRedAlliance ? 4 : 7;
      Pose2d tagPose = RobotContainer.aprilTagFieldLayout.getTagPose(tagID).get().toPose2d();
      Pose2d robotPose = RobotContainer.poseEstimation.getEstimatedPose();

      z = tagPose.getX() - robotPose.getX() + 0.07;
      x = (tagPose.getY() - 0.11) - robotPose.getY();
    }

    double hypot = Math.hypot(x, z);

    double angle = Math.atan((FieldConstants.SPEAKER_HEIGHT - ShooterConstants.FLOOR_TO_SHOOTER) / hypot);
    return Units.radiansToDegrees(angle);
  }

  public boolean hasPriorityTarget(boolean isRedAlliance) {
    if(isRedAlliance){
      return llTable.getEntry("tid").getDouble(0) == 4;
    }
    else{
      return llTable.getEntry("tid").getDouble(0) == 7;
    }
  }

  @Override
  public void setPipeline(int index) {
    llTable.getEntry("pipeline").setNumber(index);
  }
}
