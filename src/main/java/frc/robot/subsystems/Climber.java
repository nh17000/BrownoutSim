// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkBase.ControlType;
// import com.revrobotics.spark.SparkClosedLoopController;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.lib.drivers.PearadoxSparkMax;
// import frc.lib.util.SmarterDashboard;
// import frc.robot.RobotContainer;
// import frc.robot.Constants.ClimberConstants;

// public class Climber extends SubsystemBase {

//   private static PearadoxSparkMax leftClimber;
//   private static PearadoxSparkMax rightClimber;

//   private static SparkClosedLoopController leftClimberController;
//   private static SparkClosedLoopController rightClimberController;

//   private static RelativeEncoder leftClimberEncoder;
//   private static RelativeEncoder rightClimberEncoder;

//   private static Climber climber = new Climber();

//   private double climberAdjust = 0;
//   private boolean zeroing = false;
//   private int climbSequenceStep = -1;

//   public static Climber getInstance(){
//     return climber;
//   }

//   public Climber() {
//     leftClimber = new PearadoxSparkMax(ClimberConstants.LEFT_CLIMBER_ID, MotorType.kBrushless, IdleMode.kBrake, 60, true,
//       ClimberConstants.CLIMBER_kP, ClimberConstants.CLIMBER_kI, ClimberConstants.CLIMBER_kD, 
//       ClimberConstants.CLIMBER_MIN_OUTPUT, ClimberConstants.CLIMBER_MAX_OUTPUT);// Check for inversion
//     rightClimber = new PearadoxSparkMax(ClimberConstants.RIGHT_CLIMBER_ID, MotorType.kBrushless, IdleMode.kBrake, 60, false,
//       ClimberConstants.CLIMBER_kP, ClimberConstants.CLIMBER_kI, ClimberConstants.CLIMBER_kD, 
//       ClimberConstants.CLIMBER_MIN_OUTPUT, ClimberConstants.CLIMBER_MAX_OUTPUT);// Check for inversion

//     leftClimberController = leftClimber.getClosedLoopController();
//     rightClimberController = rightClimber.getClosedLoopController();
//     leftClimberEncoder = leftClimber.getEncoder();
//     rightClimberEncoder = rightClimber.getEncoder();
//   }

//   @Override
//   public void periodic() {
//     SmarterDashboard.putNumber("Left Climber Position", getLeftPosition(), "Climber");
//     SmarterDashboard.putNumber("Right Climber Position", getRightPosition(), "Climber");
//     SmarterDashboard.putNumber("Left Climber Current", leftClimber.getOutputCurrent(), "Climber");
//     SmarterDashboard.putNumber("Right Climber Current", rightClimber.getOutputCurrent(), "Climber");
//     SmarterDashboard.putNumber("Climb Sequence Step", climbSequenceStep, "Climber");
//     SmarterDashboard.putNumber("Climber Adjust", climberAdjust, "Climber");

//     if(RobotContainer.opController.getLeftTriggerAxis() > 0.95){
//       climberAdjust -= 0.05;
//     }
//     else if(RobotContainer.opController.getRightTriggerAxis() > 0.95){
//       climberAdjust += 0.05;
      
//     }
//   }

//   public void setClimberPosition(double reference){
//     leftClimberController.setReference(reference + climberAdjust, ControlType.kPosition, 0);
//     rightClimberController.setReference(reference + climberAdjust, ControlType.kPosition, 0);
//   }

//   public void setZeroing(boolean zeroing){
//     this.zeroing = zeroing;
//   }

//   public void zeroClimber(){
//     leftClimber.set(-0.5);
//     rightClimber.set(-0.5);
//   }

//   // public void setCurrentLimit(int limit){
//   //   leftClimber.setSmartCurrentLimit(limit);
//   //   rightClimber.setSmartCurrentLimit(limit);
//   // }

//   public void resetEncoders(){
//     leftClimberEncoder.setPosition(0);
//     rightClimberEncoder.setPosition(0);
//   }

//   public double getRightPosition(){
//     return rightClimberEncoder.getPosition();
//   }

//   public double getLeftPosition(){
//     return leftClimberEncoder.getPosition();
//   }

//   public int getClimbSequenceStep(){
//     return climbSequenceStep;
//   }

//   public void resetClimbSequence(){
//     if(climbSequenceStep == 0){
//       climbSequenceStep = -1;
//     }
//     else{
//       climbSequenceStep = 0;
//     }
//   }

//   public void nextClimbSequenceStep(){
//     climbSequenceStep++;
//   }

//   public boolean getZeroing(){
//     return zeroing;
//   }

//   public void setClimbingMode() {
//     leftClimber.set(0.75);
//     rightClimber.set(0.75);
//   }

//   public void setStoppedMode() {
//     leftClimber.stopMotor();
//     rightClimber.stopMotor();
//   }

//   public void setLoweringMode() {
//     leftClimber.set(-0.75);
//     rightClimber.set(-0.75);
//   }

//   public double getOutputCurrent() {
//     double c1 = leftClimber.getOutputCurrent();
//     double c2 = rightClimber.getOutputCurrent();
//     return (c1 + c2) / 2.0;
//   }
// }