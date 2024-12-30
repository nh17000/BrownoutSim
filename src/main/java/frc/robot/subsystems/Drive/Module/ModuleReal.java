package frc.robot.subsystems.Drive.Module;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
// import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.drivers.PearadoxTalonFX;
// import frc.lib.util.PhoenixOdometryThread;
import frc.robot.Constants.SwerveConstants;

public class ModuleReal implements ModuleIO {
  private PearadoxTalonFX driveMotor;
  private PearadoxTalonFX turnMotor;

  private PIDController turnPIDController;
  private CANcoder absoluteEncoder;

  private StatusSignal<Angle> drivePosition;
  private StatusSignal<Angle> turnPosition;
  private StatusSignal<Angle> absoluteEncoderAngle;
  private StatusSignal<AngularVelocity> driveVelocity;
  private StatusSignal<AngularVelocity> turnVelocity;
  private StatusSignal<Current> driveCurrent;
  private StatusSignal<Current> turnCurrent;
  private StatusSignal<Voltage> driveVolts;
  private StatusSignal<Voltage> turnVolts;
  

  // private DutyCycleOut driveMotorRequest = new DutyCycleOut(0.0);
  // private DutyCycleOut turnMotorRequest = new DutyCycleOut(0.0);
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

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

    driveMotor = new PearadoxTalonFX(driveMotorId, NeutralModeValue.Coast, SwerveConstants.DRIVE_CURRENT_LIMIT, driveMotorReversed);
    turnMotor = new PearadoxTalonFX(turnMotorId, NeutralModeValue.Brake, SwerveConstants.TURN_CURRENT_LIMIT, turnMotorReversed);

    turnPIDController = new PIDController(SwerveConstants.KP_TURNING, 0, 0);
    turnPIDController.enableContinuousInput(-Math.PI, Math.PI);

    // Create drive status signals
    drivePosition = driveMotor.getPosition();
    // drivePositionQueue = PhoenixOdometryThread.getInstance().registerSignal(driveTalon.getPosition());
    driveVelocity = driveMotor.getVelocity();
    driveVolts = driveMotor.getMotorVoltage();
    driveCurrent = driveMotor.getStatorCurrent();

    // Create turn status signals
    absoluteEncoderAngle = absoluteEncoder.getAbsolutePosition();
    turnPosition = turnMotor.getPosition();
    // turnPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(turnTalon.getPosition());
    turnVelocity = turnMotor.getVelocity();
    turnVolts = turnMotor.getMotorVoltage();
    turnCurrent = turnMotor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(SwerveConstants.ODOMETRY_FREQUENCY, drivePosition, driveVelocity);
    BaseStatusSignal.setUpdateFrequencyForAll(
      50.0, drivePosition, driveVelocity, turnPosition, turnVelocity, 
      absoluteEncoderAngle, driveCurrent, turnCurrent, driveVolts, turnVolts);

    driveMotor.optimizeBusUtilization();
    turnMotor.optimizeBusUtilization();

    this.absoluteEncoderOffset = absoluteEncoderOffset;

    resetEncoders();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Refresh all signals
    // var driveStatus = 
    BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveVolts, driveCurrent);
    // var turnStatus = 
    BaseStatusSignal.refreshAll(turnPosition, turnVelocity, turnVolts, turnCurrent);
    // var turnEncoderStatus = 
    BaseStatusSignal.refreshAll(absoluteEncoderAngle);

    inputs.driveCurrent = driveCurrent.getValueAsDouble();   
    inputs.drivePosition = drivePosition.getValueAsDouble() * SwerveConstants.DRIVE_MOTOR_PCONVERSION;
    inputs.driveVelocity = driveVelocity.getValueAsDouble() * SwerveConstants.DRIVE_MOTOR_VCONVERSION;
    inputs.driveVolts = driveVolts.getValueAsDouble();
    
    inputs.turnCurrent = turnCurrent.getValueAsDouble();
    inputs.turnPosition = turnPosition.getValueAsDouble() * SwerveConstants.TURN_MOTOR_PCONVERSION;
    inputs.turnVelocity = turnVelocity.getValueAsDouble() * SwerveConstants.TURN_MOTOR_VCONVERSION;
    inputs.turnVolts = turnVolts.getValueAsDouble();

    inputs.absoluteWheelAngleDeg = absoluteEncoderAngle.getValueAsDouble();
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
  public void setVelocity(double wheelRadsPerSec) {
    // driveMotor.setControl(driveMotorRequest.withOutput(speed)); // -1 to 1
    double rotPerSec = Units.radiansToRotations(wheelRadsPerSec) * SwerveConstants.DRIVE_MOTOR_GEAR_RATIO;
    driveMotor.setControl(velocityVoltageRequest.withVelocity(rotPerSec));
  }

  @Override
  public void setAngle(Rotation2d angle) {
    // turnMotor.setControl(turnMotorRequest.withOutput(turnPIDController.calculate(currPos, angleRadians)));
    turnMotor.setControl(positionVoltageRequest.withPosition(angle.getRotations()));
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveMotor.setControl(voltageRequest.withOutput(output));
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnMotor.setControl(voltageRequest.withOutput(output));
  }

  @Override
  public int getDeviceID() {
    return driveMotor.getDeviceID();
  }
}
