// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

// import com.revrobotics.CANSparkBase.ControlType;
// import com.revrobotics.CANSparkBase.IdleMode;
// import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.Map;

// import com.ctre.phoenix6.controls.VoltageOut;
// import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkPIDController;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.lib.drivers.PearadoxSparkFlex;
// import frc.lib.drivers.PearadoxTalonFX;
import frc.lib.util.LerpTable;
import frc.lib.util.SmarterDashboard;
import frc.robot.RobotContainer;
// import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
// import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Drive.Drivetrain;
import frc.robot.subsystems.Transport.Transport;

public class Shooter extends SubsystemBase {
  // private PearadoxTalonFX leftShooter;
  // private PearadoxTalonFX rightShooter;
  
  // private PearadoxSparkFlex pivot;

  // private RelativeEncoder pivotEncoder;

  // private SparkPIDController pivotController;

  private boolean zeroing = false;

  // private static final NetworkTable llTable = NetworkTableInstance.getDefault().getTable(VisionConstants.SHOOTER_LL_NAME);

  private double pivotIntendedPos;
  private double pivotAdjust = 0;
  // private double[] botpose_targetspace = new double[6];
  public static final Drivetrain drivetrain = Drivetrain.getInstance();

  private LerpTable pivotLerp = new LerpTable();
  private LerpTable shooterLerp = new LerpTable();

  private static Shooter SHOOTER_KRAKEN;

  public static Shooter createInstance(ShooterIO io) {
    if (SHOOTER_KRAKEN == null) {
      SHOOTER_KRAKEN = new Shooter(io);
    } else {
      throw new IllegalStateException("Shooter instance already created!");
    }
    return SHOOTER_KRAKEN;
  }

  public static Shooter getInstance(){
    if (SHOOTER_KRAKEN == null) {
      throw new IllegalStateException("Shooter instance not created");
    }
    return SHOOTER_KRAKEN;
  }

  private ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public enum ShooterMode{
    Auto, Manual, SourcePassing, AmpPassing, Speaker, Outtake
  }

  private ShooterMode shooterMode = ShooterMode.Auto;

  private Transport transport = Transport.getInstance();

  public static ShuffleboardTab driverTab;
  private GenericEntry leftShooterSpeedEntry;
  private GenericEntry rightShooterSpeedEntry;
  private GenericEntry shooterModeEntry;
  private GenericEntry pivotAdjustEntry;
  private GenericEntry hasPriorityTargetEntry;
  // private VoltageOut voltage_request = new VoltageOut(0);

  public Shooter(ShooterIO io) {
    // leftShooter = new PearadoxTalonFX(ShooterConstants.LEFT_SHOOTER_ID, NeutralModeValue.Coast, 50, true); 
    
    // rightShooter = new PearadoxTalonFX(ShooterConstants.RIGHT_SHOOTER_ID, NeutralModeValue.Coast, 50, false);

    // pivot = new PearadoxSparkFlex(ShooterConstants.PIVOT_ID, MotorType.kBrushless, IdleMode.kBrake, 50, true,
    //   ShooterConstants.PIVOT_kP, ShooterConstants.PIVOT_kI, ShooterConstants.PIVOT_kD,
    //   ShooterConstants.PIVOT_MIN_OUTPUT, ShooterConstants.PIVOT_MAX_OUTPUT);

    // pivotEncoder = pivot.getEncoder();

    // pivotController = pivot.getPIDController();

    //Red
    pivotLerp.addPoint(52, 32.4); //0ft
    pivotLerp.addPoint(46.7, 29.57);
    pivotLerp.addPoint(41.8, 27.2);
    pivotLerp.addPoint(38.7, 25.6);
    pivotLerp.addPoint(35.2, 23.26);
    pivotLerp.addPoint(32.4, 19.93); //5ft
    pivotLerp.addPoint(29.6, 18.93);
    pivotLerp.addPoint(27.7, 17.46);
    pivotLerp.addPoint(25.5, 16.55);
    pivotLerp.addPoint(24, 15.3);
    pivotLerp.addPoint(22.4, 14.2); //10ft // WCMP: Subtracted 0.5 from this point
    pivotLerp.addPoint(21.4, 13.5);
    pivotLerp.addPoint(20.3, 12.7); // WCMP: Subtracted 1.0 from this point and some points below
    pivotLerp.addPoint(19.1, 12.1); 
    pivotLerp.addPoint(18.3, 11.65); //WCMP: Subtracted 0.05 from this point and some points below
    pivotLerp.addPoint(17.4, 11.15); //15ft
    pivotLerp.addPoint(16.4, 10.35);
    pivotLerp.addPoint(15.9, 9.65);
    pivotLerp.addPoint(15.35, 9.45);
    pivotLerp.addPoint(14.9, 9.15);
    pivotLerp.addPoint(14, 8.75);  //20ft
    pivotLerp.addPoint(13.5, 8.3); //WCMP: No more changes here and below
    pivotLerp.addPoint(13, 7.9);
    pivotLerp.addPoint(12.6, 7.7);
    pivotLerp.addPoint(12, 7.4);
    pivotLerp.addPoint(11.5, 7.15); //25ft

    shooterLerp.addPoint(53, 8);
    shooterLerp.addPoint(47, 8);
    shooterLerp.addPoint(41, 8.2);
    shooterLerp.addPoint(35, 8.5);
    shooterLerp.addPoint(29, 8.75);
    shooterLerp.addPoint(23, 9);
    shooterLerp.addPoint(18, 9.25);
    shooterLerp.addPoint(17, 9.5);
    shooterLerp.addPoint(15, 9.75);
    shooterLerp.addPoint(13, 10.25);
    shooterLerp.addPoint(11.5, 10.75);

    driverTab = Shuffleboard.getTab("Driver");
    leftShooterSpeedEntry = driverTab.add("Left Shooter Speed", 7)
      .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 12)).withPosition(2, 0).getEntry();
    rightShooterSpeedEntry = driverTab.add("Right Shooter Speed", 4)
      .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 12)).withPosition(2, 1).getEntry();
    shooterModeEntry = driverTab.add("Shooter Mode", shooterMode.toString()).withPosition(2, 2).getEntry();
    pivotAdjustEntry = driverTab.add("Shooter Pivot Adjust", pivotAdjust).withPosition(3, 2).getEntry();
    hasPriorityTargetEntry = driverTab.add("Shooter Has Priority Target", hasPriorityTarget()).withPosition(2, 3).getEntry();

    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs); 
    
    // This method will be called once per scheduler run
    SmarterDashboard.putString("Shooter Mode", getShooterMode().toString(), "Shooter");
    SmarterDashboard.putNumber("Shooter Pivot Intended Position", pivotIntendedPos, "Shooter");
    SmarterDashboard.putNumber("Shooter Pivot Intended Angle", calculatePivotAngle(), "Shooter");  
    SmarterDashboard.putBoolean("Shooter Has Priority Target", hasPriorityTarget(), "Shooter"); 
    SmarterDashboard.putNumber("Shooter Pivot Adjust", pivotAdjust, "Shooter");

    // SmarterDashboard.putNumber("Shooter Pivot Position", pivotEncoder.getPosition(), "Shooter");
    // SmarterDashboard.putNumber("Shooter Pivot Current", pivot.getOutputCurrent(), "Shooter");
    // SmarterDashboard.putNumber("Left Shooter Speed", leftShooter.getVelocity().getValueAsDouble() * 60.0, "Shooter");
    // SmarterDashboard.putNumber("Right Shooter Speed", rightShooter.getVelocity().getValueAsDouble() * 60.0, "Shooter");
    // SmarterDashboard.putNumber("Right Shooter Stator Current", rightShooter.getStatorCurrent().getValueAsDouble(), "Shooter");
    // SmarterDashboard.putNumber("Left Shooter Stator Current", leftShooter.getStatorCurrent().getValueAsDouble(), "Shooter");
    // SmarterDashboard.putNumber("Right Shooter Supply Current", rightShooter.getSupplyCurrent().getValueAsDouble(), "Shooter");
    // SmarterDashboard.putNumber("Left Shooter Supply Current", leftShooter.getSupplyCurrent().getValueAsDouble(), "Shooter");

    shooterModeEntry.setString(shooterMode.toString());
    pivotAdjustEntry.setDouble(pivotAdjust);
    hasPriorityTargetEntry.setBoolean(hasPriorityTarget());
  }

  public void shooterHold(){
    double shooterVoltage = shooterLerp.interpolate(calculatePivotAngle());
    SmarterDashboard.putNumber("Shooter Auto Voltage", shooterVoltage, "Shooter");

    // if(RobotContainer.climber.getClimbSequenceStep() >= 0){
    //   leftShooter.setControl(voltage_request.withOutput(0));

    //   rightShooter.setControl(voltage_request.withOutput(0));
    // }
    if ((!transport.hasNote() && (((System.currentTimeMillis()) - transport.getRequestedShootTime()) > 100)) && !RobotContainer.opController.getRightBumperButton()) {
      io.setShooterVolts(0.0, 0.0);
    }
    else if(RobotContainer.driverController.getLeftTriggerAxis() >= 0.95){ //Amp
      io.setShooterVolts(ShooterConstants.AMP_VOLTAGE, ShooterConstants.AMP_VOLTAGE);
    }
    else if(shooterMode == ShooterMode.SourcePassing){
      io.setShooterVolts(ShooterConstants.PASSING_VOLTAGE, 
          ShooterConstants.PASSING_VOLTAGE - ShooterConstants.LEFT_TO_RIGHT_VOLTAGE_OFFSET);
    }
    else if(shooterMode == ShooterMode.AmpPassing){
      io.setShooterVolts(ShooterConstants.PASSING_VOLTAGE - 1, 
          ShooterConstants.PASSING_VOLTAGE - ShooterConstants.LEFT_TO_RIGHT_VOLTAGE_OFFSET - 1);
    }
    else if(shooterMode == ShooterMode.Speaker){
      io.setShooterVolts(ShooterConstants.SPEAKER_VOLTAGE, 
          ShooterConstants.SPEAKER_VOLTAGE - ShooterConstants.LEFT_TO_RIGHT_VOLTAGE_OFFSET);
    }
    else if(shooterMode == ShooterMode.Outtake){
      io.setShooterVolts(ShooterConstants.OUTAKE_VOLTAGE, ShooterConstants.OUTAKE_VOLTAGE);
    }
    else if(shooterMode == ShooterMode.Manual){
      io.setShooterVolts(leftShooterSpeedEntry.getDouble(7), 
          rightShooterSpeedEntry.getDouble(4));
    }
    else{
      io.setShooterVolts(shooterVoltage, shooterVoltage - ShooterConstants.LEFT_TO_RIGHT_VOLTAGE_OFFSET);
    }
  }

  public void setShooterAuto(double speed){
    setAutoMode();
    io.setShooterSpeed(speed, speed);
  }

  public void pivotHold(){
    if(zeroing){
      io.setPivotSpeed(ShooterConstants.PIVOT_ZEROING_SPEED);
    }
    // else if(RobotContainer.climber.getClimbSequenceStep() >= 0){
    //   pivotController.setReference(
    //     2,
    //     ControlType.kPosition,
    //     0);

    //   pivotPosition = 2;
    // }
    else if(RobotContainer.driverController.getLeftTriggerAxis() >= 0.95){
      io.setPivotReference(ShooterConstants.AMP_PIVOT_POSITION);

      pivotIntendedPos = ShooterConstants.AMP_PIVOT_POSITION;
    }
    else if(shooterMode == ShooterMode.SourcePassing || shooterMode == ShooterMode.AmpPassing){
      io.setPivotReference(ShooterConstants.PASSING_PIVOT_POSITION);

      pivotIntendedPos = ShooterConstants.PASSING_PIVOT_POSITION;
    }
    else if(shooterMode == ShooterMode.Speaker){
      io.setPivotReference(ShooterConstants.SPEAKER_PIVOT_POSITION);

      pivotIntendedPos = ShooterConstants.SPEAKER_PIVOT_POSITION;
    }
    else if(shooterMode == ShooterMode.Outtake){
      io.setPivotReference(ShooterConstants.PIVOT_OUTAKE_POSITION);

      pivotIntendedPos = ShooterConstants.PIVOT_OUTAKE_POSITION;
    }
    else{
      if(shooterMode == ShooterMode.Auto){
        setPivotAngle(calculatePivotAngle());
      }

      io.setPivotReference(pivotIntendedPos + pivotAdjust);
    }

    if(RobotContainer.opController.getPOV() == 0){
      pivotAdjust += 0.1;
    }
    else if(RobotContainer.opController.getPOV() == 180){
      pivotAdjust -= 0.1;
    }
  }

  public void setZeroing(boolean zeroing){
    this.zeroing = zeroing;
  }

  public void resetPivotEncoder(){
    io.resetPivotEncoder();
  }

  public void setBrakeMode(boolean brake){
    io.setBrakeMode(brake);
    // if(brake){
    //   leftShooter.setNeutralMode(NeutralModeValue.Brake);
    //   rightShooter.setNeutralMode(NeutralModeValue.Brake);
      // pivot.setBrakeMode(IdleMode.kBrake);

    // }
    // else{
    //   leftShooter.setNeutralMode(NeutralModeValue.Coast);
    //   rightShooter.setNeutralMode(NeutralModeValue.Coast);
    // }
  }

  public void setPivot(double speed){
    io.setPivotSpeed(speed);
  }

  public double getPivotCurrent(){
    return inputs.shooterPivotCurrent;
  }

  public double calculatePivotAngle(){
    return io.calculatePivotAngle(isRedAlliance());
  }

  public void setPivotAngle(double angle){
    pivotIntendedPos = pivotLerp.interpolate(angle);
  }

  public void setPivotPosition(){
    io.setPivotReference(pivotIntendedPos + pivotAdjust);
  }

  // public double getNoteVelocity(){
  //   return 2 * (((leftEncoder.getVelocity() + rightEncoder.getVelocity()) / 2) * 2 * Math.PI * Units.inchesToMeters(1.5) / 60);
  // }

  public boolean hasPriorityTarget(){
    if (this.io != null) { return io.hasPriorityTarget(isRedAlliance()); }
    return false;
  }

  public void setPipeline(int index){
    io.setPipeline(index);
  }

  public void setPivotIntendedPos(double position){
    pivotIntendedPos = position;
  }

  public ShooterMode getShooterMode(){
    return shooterMode;
  }

  public void setAutoMode(){
    shooterMode = ShooterMode.Auto;
  }

  public void setManualMode(){
    shooterMode = ShooterMode.Manual;
  }

  public void setSourcePassingMode(){
    shooterMode = ShooterMode.SourcePassing;
  }

  public void setAmpPassingMode(){
    shooterMode = ShooterMode.AmpPassing;
  }

  public void setSpeakerMode(){
    shooterMode = ShooterMode.Speaker;
  }

  public void setOuttakeMode(){
    shooterMode = ShooterMode.Outtake;
  }

  public boolean isRedAlliance(){
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
        return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
  }

  public boolean readyToShoot() {
    return (Math.abs(pivotIntendedPos + pivotAdjust - inputs.shooterPivotPos) <= 0.9);
  }

  public void setCurrentLimit(double limit){
    io.setCurrentLimit(limit);
  }

  public void shootNote() {
    io.shootNote();
  }
}
