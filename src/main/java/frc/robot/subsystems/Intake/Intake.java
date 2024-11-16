// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

// import com.revrobotics.CANSparkBase.IdleMode;
// import com.revrobotics.CANSparkLowLevel.MotorType;

// import edu.wpi.first.math.filter.Debouncer;
// import edu.wpi.first.math.filter.Debouncer.DebounceType;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.lib.drivers.PearadoxSparkMax;
// import frc.lib.util.SmarterDashboard;
// import frc.robot.Constants.IntakeConstants;
// import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  // private PearadoxSparkMax utbRoller;

  private boolean rumbled = false;

  // private Debouncer debouncer;

  // private static final NetworkTable llTable = NetworkTableInstance.getDefault().getTable(VisionConstants.INTAKE_LL_NAME);

  private static Intake INTAKE;

  public static Intake createInstance(IntakeIO io) {
    if (INTAKE == null) {
      INTAKE = new Intake(io);
    } else {
      throw new IllegalStateException("Intake instance already created!");
    }
    return INTAKE;
  }

  public static Intake getInstance() {
    if (INTAKE == null) {
      throw new IllegalStateException("Intake instance not created");
    }
    return INTAKE;
  }
  
  private IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  /** Creates a new Intake. */
  private Intake(IntakeIO io) {
    // utbRoller = new PearadoxSparkMax(IntakeConstants.UTB_ROLLER_ID, MotorType.kBrushless, IdleMode.kCoast, 80, false);
    // debouncer = new Debouncer(0.5, DebounceType.kRising);
    this.io = io;
  }

  @Override
  public void periodic() {
    // SmarterDashboard.putNumber("Intake Current", utbRoller.getOutputCurrent(), "Intake");
    // SmarterDashboard.putBoolean("Intake Has Target", hasTarget(), "Intake");

    io.updateInputs(inputs);

    if(!rumbled && inputs.intakeCurrent > IntakeConstants.RUMBLE_AT_CURRENT){
      CommandScheduler.getInstance().schedule(rumbleController());
      rumbled = true;
    }
    if(rumbled && !(inputs.intakeCurrent > IntakeConstants.RUMBLE_AT_CURRENT)){
      rumbled = false;
    }
  }

  public void utbIntakeIn(){
    io.set(IntakeConstants.INTAKE_IN_SPEED);
  }

  public void utbIntakeOut(){
    io.set(IntakeConstants.INTAKE_OUT_SPEED);
  }

  public void utbIntakeStop(){
    io.set(0);
  }

  public Command rumbleController(){
    return new InstantCommand(() -> RobotContainer.driverController.setRumble(RumbleType.kBothRumble, 0.75))
      .andThen(new InstantCommand(() -> RobotContainer.opController.setRumble(RumbleType.kBothRumble, 0.75)))
      .andThen(new WaitCommand(0.25))
      .andThen(new InstantCommand(() -> RobotContainer.driverController.setRumble(RumbleType.kBothRumble, 0)))
      .andThen(new InstantCommand(() -> RobotContainer.opController.setRumble(RumbleType.kBothRumble, 0)));
  }

  public boolean hasTarget(){
    return inputs.hasTarget;
  }

  public boolean obtainNoteFromIntakeSim() {
    if (Robot.isReal()) return false;
    return io.obtainGamePieceFromIntake();
  }

  // because the intake sim is 
  public boolean simHasNote() {
    if (Robot.isReal()) return false;
    return io.simHasNote();
  }
}
