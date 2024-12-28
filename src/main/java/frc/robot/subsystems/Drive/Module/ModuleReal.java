package frc.robot.subsystems.Drive.Module;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import frc.lib.drivers.PearadoxTalonFX;
import frc.robot.Constants.SwerveConstants;

public class ModuleReal implements ModuleIO {
  private PearadoxTalonFX driveMotor;
  private PearadoxTalonFX turnMotor;

  private PIDController turnPIDController;
  private CANcoder absoluteEncoder;

  private StatusSignal<Double> drivePosition;
  private StatusSignal<Double> driveVelocity;
  private StatusSignal<Double> turnPosition;
  private StatusSignal<Double> turnVelocity;
  private StatusSignal<Double> absoluteEncoderAngle;
  private StatusSignal<Double> driveCurrent;
  private StatusSignal<Double> turnCurrent;

  private DutyCycleOut driveMotorRequest = new DutyCycleOut(0.0);
  private DutyCycleOut turnMotorRequest = new DutyCycleOut(0.0);

  private double absoluteEncoderOffset;

  public ModuleReal(int index) {
    switch (index) {
      case 0:
        configureHardware(
        SwerveConstants.LEFT_FRONT_DRIVE_ID, 
        SwerveConstants.LEFT_FRONT_TURN_ID, 
        false, 
        true, 
        SwerveConstants.LEFT_FRONT_CANCODER_ID, 
        SwerveConstants.LEFT_FRONT_OFFSET);
      break;
      
      case 1:
        configureHardware(
          SwerveConstants.RIGHT_FRONT_DRIVE_ID, 
          SwerveConstants.RIGHT_FRONT_TURN_ID, 
          false, 
          false, 
          SwerveConstants.RIGHT_FRONT_CANCODER_ID, 
          SwerveConstants.RIGHT_FRONT_OFFSET);
        break;

      case 2:
        configureHardware(
          SwerveConstants.LEFT_BACK_DRIVE_ID, 
          SwerveConstants.LEFT_BACK_TURN_ID, 
          false, 
          true, 
          SwerveConstants.LEFT_BACK_CANCODER_ID, 
          SwerveConstants.LEFT_BACK_OFFSET);
        break;

      default:
        configureHardware(
          SwerveConstants.RIGHT_BACK_DRIVE_ID, 
          SwerveConstants.RIGHT_BACK_TURN_ID, 
          false, 
          true, 
          SwerveConstants.RIGHT_BACK_CANCODER_ID, 
          SwerveConstants.RIGHT_BACK_OFFSET);
    }
  }

  public void configureHardware(int driveMotorId, int turnMotorId, boolean driveMotorReversed, boolean turnMotorReversed,
      int absoluteEncoderId, double absoluteEncoderOffset) {
    absoluteEncoder = new CANcoder(absoluteEncoderId);

    driveMotor = new PearadoxTalonFX(driveMotorId, NeutralModeValue.Coast, 45, driveMotorReversed);
    turnMotor = new PearadoxTalonFX(turnMotorId, NeutralModeValue.Brake, 25, turnMotorReversed);

    turnPIDController = new PIDController(SwerveConstants.KP_TURNING, 0, 0);
    turnPIDController.enableContinuousInput(-Math.PI, Math.PI);

    drivePosition = driveMotor.getPosition();
    driveVelocity = driveMotor.getVelocity();
    turnPosition = turnMotor.getPosition();
    turnVelocity = turnMotor.getVelocity();
    absoluteEncoderAngle = absoluteEncoder.getAbsolutePosition();
    driveCurrent = driveMotor.getStatorCurrent();
    turnCurrent = turnMotor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(50, drivePosition, driveVelocity, turnPosition, turnVelocity, absoluteEncoderAngle, driveCurrent, turnCurrent);
    driveMotor.optimizeBusUtilization();
    turnMotor.optimizeBusUtilization();

    this.absoluteEncoderOffset = absoluteEncoderOffset;

    resetEncoders();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.driveCurrent = driveMotor.getStatorCurrent().getValueAsDouble();   
    inputs.drivePosition = driveMotor.getPosition().getValueAsDouble() * SwerveConstants.DRIVE_MOTOR_PCONVERSION;
    inputs.driveVelocity = driveMotor.getVelocity().getValueAsDouble() * SwerveConstants.DRIVE_MOTOR_VCONVERSION;
    
    inputs.turnCurrent = turnMotor.getStatorCurrent().getValueAsDouble();
    inputs.turnPosition = turnMotor.getPosition().getValueAsDouble() * SwerveConstants.TURN_MOTOR_PCONVERSION;
    inputs.turnVelocity = turnMotor.getVelocity().getValueAsDouble() * SwerveConstants.TURN_MOTOR_VCONVERSION;

    inputs.absoluteWheelAngleDeg = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
    inputs.turnAngle = getTurnAngle();
    inputs.offset = absoluteEncoderOffset;
  }

  @Override
  public void setDriveBrake(boolean brake) {
    if (brake) {
      driveMotor.setNeutralMode(NeutralModeValue.Brake);
      turnMotor.setNeutralMode(NeutralModeValue.Coast);
    } else {
      driveMotor.setNeutralMode(NeutralModeValue.Coast);
      turnMotor.setNeutralMode(NeutralModeValue.Coast);
    }
  }

  public double getTurnAngle() {
    double angle = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
    angle -= absoluteEncoderOffset;
    angle *= (2 * Math.PI);
    return angle;
  }

  @Override
  public void resetEncoders() {
    driveMotor.setPosition(0);
    turnMotor.setPosition(getTurnAngle() / SwerveConstants.TURN_MOTOR_PCONVERSION);
  }

  @Override
  public void setSpeed(double speed) {
    driveMotor.setControl(driveMotorRequest.withOutput(speed));
  }

  @Override
  public void setAngle(double currPos, double angleRadians) {
    turnMotor.setControl(turnMotorRequest.withOutput(turnPIDController.calculate(currPos, angleRadians)));
  }

  @Override
  public void stop() {
    driveMotor.setControl(driveMotorRequest.withOutput(0));
    turnMotor.setControl(turnMotorRequest.withOutput(0));
  }

  @Override
  public int getDeviceID() {
    return driveMotor.getDeviceID();
  }
}
