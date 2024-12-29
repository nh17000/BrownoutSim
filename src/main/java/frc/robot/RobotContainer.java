// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.List;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.drivers.vision.PoseEstimation;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.AmpBarHold;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.IntakeHold;
import frc.robot.commands.Outtake;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShooterHold;
import frc.robot.commands.SourceAutoAlign;
import frc.robot.commands.SwerveDrive;
// import frc.robot.subsystems.Climber;
// import frc.robot.subsystems.LEDStrip;
import frc.robot.subsystems.AmpBar.AmpBar;
import frc.robot.subsystems.Drive.Drivetrain;
import frc.robot.subsystems.Drive.Gyro.GyroIOPigeon2;
import frc.robot.subsystems.Drive.Gyro.GyroIOSim;
import frc.robot.subsystems.Drive.Module.ModuleReal;
import frc.robot.subsystems.Drive.Module.ModuleSim;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeReal;
import frc.robot.subsystems.Intake.IntakeSim;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterReal;
import frc.robot.subsystems.Shooter.ShooterSim;
import frc.robot.subsystems.Transport.Transport;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static Drivetrain drivetrain;
  public static Intake intake;
  public static final Transport transport = Transport.getInstance();
  // public static final Climber climber = Climber.getInstance();
  public static final AmpBar ampBar = AmpBar.getInstance();
  public static Shooter shooter;
  // public static final LEDStrip ledStrip = new LEDStrip(90, 0);

  private final SwerveDriveSimulation swerveDriveSimulation;
  public static boolean isSimulation = false;

  //Driver Controls
  public static final CommandXboxController commandDriverController = new CommandXboxController(IOConstants.DRIVER_CONTROLLER_PORT);
  public static final XboxController driverController = commandDriverController.getHID();

  private final JoystickButton resetHeading_Start = new JoystickButton(driverController, XboxController.Button.kStart.value);
  private final JoystickButton shoot_RB = new JoystickButton(driverController, XboxController.Button.kRightBumper.value);
  private final JoystickButton zeroingShooter_X = new JoystickButton(driverController, XboxController.Button.kX.value);
  private final JoystickButton outtake_B = new JoystickButton(driverController, XboxController.Button.kB.value);
  private final JoystickButton turnToApril_LB = new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);
  private final JoystickButton turnToNote_LS = new JoystickButton(driverController, XboxController.Button.kLeftStick.value);
  // private final JoystickButton ampAlign_Y = new JoystickButton(driverController, XboxController.Button.kY.value);
  //Operator Controls
  public static final CommandXboxController commandOpController = new CommandXboxController(IOConstants.OP_CONTROLLER_PORT);
  public static final XboxController opController = commandOpController.getHID();  

  private final JoystickButton shooterAutoMode_A = new JoystickButton(opController, XboxController.Button.kA.value);
  private final JoystickButton shooterSourcePassingMode_Y = new JoystickButton(opController, XboxController.Button.kY.value);
  private final JoystickButton shooterAmpPassingMode_Start = new JoystickButton(opController, XboxController.Button.kStart.value);
  private final JoystickButton shooterManualMode_B = new JoystickButton(opController, XboxController.Button.kB.value);
  private final JoystickButton shooterSpeakerMode_X = new JoystickButton(opController, XboxController.Button.kX.value);
  // private final JoystickButton resetClimbSequence_LB = new JoystickButton(opController, XboxController.Button.kLeftBumper.value);
  // private final JoystickButton nextClimbSequenceStep_RB = new JoystickButton(opController, XboxController.Button.kRightBumper.value);

  //Pose Estimation
  public static PoseEstimation poseEstimation;
  public static AprilTagFieldLayout aprilTagFieldLayout;

  //Shuffleboard
  public static final ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
  private SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. 
   * @throws IOException */
  public RobotContainer() throws IOException {
    aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);

    if (Robot.isReal()) {
      this.swerveDriveSimulation = null;      
      drivetrain = Drivetrain.createInstance(
          new GyroIOPigeon2(), 
          new ModuleReal(0),
          new ModuleReal(1),
          new ModuleReal(2),
          new ModuleReal(3));

      intake = Intake.createInstance(new IntakeReal());

      shooter = Shooter.createInstance(new ShooterReal());
    } else {  
      this.swerveDriveSimulation = new SwerveDriveSimulation(
        SwerveConstants.mapleSimConfig,
        FieldConstants.INIT_SIM_POSE); // initial starting pose on the field

      // register the drivetrain simulation
      SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSimulation);  
  
      // reset the field for auto (placing game-pieces in positions)
      SimulatedArena.getInstance().resetFieldForAuto();

      drivetrain = Drivetrain.createInstance(
        new GyroIOSim(swerveDriveSimulation.getGyroSimulation()), 
        new ModuleSim(swerveDriveSimulation.getModules()[0], 0),
        new ModuleSim(swerveDriveSimulation.getModules()[1], 1),
        new ModuleSim(swerveDriveSimulation.getModules()[2], 2),
        new ModuleSim(swerveDriveSimulation.getModules()[3], 3));
      
      intake = Intake.createInstance(new IntakeSim(swerveDriveSimulation));

      shooter = Shooter.createInstance(new ShooterSim(swerveDriveSimulation));

      DriverStation.silenceJoystickConnectionWarning(true);
    }

    registerNamedCommands();
    configureBindings();
    setDefaultCommands();
    configureAutoTab();

    poseEstimation = new PoseEstimation();
    if (Robot.isSimulation()) { drivetrain.resetPose(FieldConstants.INIT_SIM_POSE); }

    // HttpCamera httpCamera = new HttpCamera("Limelight", "http://10.54.14.11:5800");
    // CameraServer.addCamera(httpCamera);
    // driverTab.add(httpCamera).withSize(6, 4).withPosition(4, 0);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    //Driver Buttons
    resetHeading_Start.onTrue(new InstantCommand(drivetrain::zeroHeading, drivetrain));
    zeroingShooter_X.whileTrue(new RunCommand(() -> shooter.setZeroing(true)))
      .onFalse(new InstantCommand(() -> shooter.setZeroing(false))
      .andThen(new InstantCommand(() -> shooter.resetPivotEncoder())));
    shoot_RB.whileTrue(new Shoot());
    outtake_B.whileTrue(new Outtake());
    turnToApril_LB.onTrue(new InstantCommand(() -> drivetrain.setAlignMode()))
      .onFalse(new InstantCommand(() -> drivetrain.setNormalMode()));
    turnToNote_LS.onTrue(new InstantCommand(() -> drivetrain.setNoteAlignMode())
      .andThen(new InstantCommand(() -> drivetrain.changeIntakePipeline(1))))
      .onFalse(new InstantCommand(() -> drivetrain.setNormalMode()));

    //Operator Buttons
    shooterAutoMode_A.onTrue(new InstantCommand(() -> shooter.setAutoMode()));
    shooterManualMode_B.onTrue(new InstantCommand(() -> shooter.setManualMode()));
    shooterSourcePassingMode_Y.onTrue(new InstantCommand(() -> shooter.setSourcePassingMode()));
    shooterAmpPassingMode_Start.onTrue(new InstantCommand(() -> shooter.setAmpPassingMode()));
    shooterSpeakerMode_X.onTrue(new InstantCommand(() -> shooter.setSpeakerMode()));
    // resetClimbSequence_LB.whileTrue(new InstantCommand(() -> climber.setZeroing(true)))
    //   .onFalse(new InstantCommand(() -> climber.resetEncoders())
    //   .andThen(new InstantCommand(() -> climber.setZeroing(false)))
    //   .andThen(new InstantCommand(() -> climber.resetClimbSequence())));
    // nextClimbSequenceStep_RB.onTrue(new InstantCommand(() -> climber.nextClimbSequenceStep()));

    // drive to pose
    // ampAlign_Y.whileTrue(AutoBuilder.pathfindToPose(
    //     new Pose2d(2, 7.8, Rotation2d.fromDegrees(90)), 
    //     new PathConstraints(
    //         3, 
    //         3, 
    //         Units.degreesToRadians(540), 
    //         Units.degreesToRadians(720)),
    //     0,
    //     0));

    // pathfind then follow path
    // try {
    //   ampAlign_Y.whileTrue(AutoBuilder.pathfindThenFollowPath(
    //       PathPlannerPath.fromPathFile("Amp Align"), 
    //       new PathConstraints(
    //           3, // do we want it trying to align at 3m/s
    //           3, 
    //           Units.degreesToRadians(540), 
    //           Units.degreesToRadians(720))
    //       // 3.0
    //       ));
    // } catch (Exception e) {
    //   e.printStackTrace();
    // }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    drivetrain.resetAllEncoders();
    // if(!drivetrain.isRedAlliance()){
    //   drivetrain.setHeading(-60);
    // }
    // else{
    //   drivetrain.setHeading(60);
    // }
    drivetrain.setHeading(0);

    return autoChooser.getSelected();
  }

  public void registerNamedCommands(){
    NamedCommands.registerCommand("Stop Modules", new InstantCommand(() -> drivetrain.stopModules()));
    NamedCommands.registerCommand("Auto Align", new AutoAlign().withTimeout(0.5));
    NamedCommands.registerCommand("Source Auto Align", new SourceAutoAlign().withTimeout(0.7));
    NamedCommands.registerCommand("Shoot", new Shoot().withTimeout(0.2));
    NamedCommands.registerCommand("Middle Set Pivot Position", new InstantCommand(() -> shooter.setPivotIntendedPos(4.0)));
    NamedCommands.registerCommand("Set Manual Mode", new InstantCommand(() -> shooter.setManualMode()));
    NamedCommands.registerCommand("Set Auto Mode", new InstantCommand(() -> shooter.setAutoMode()));
    NamedCommands.registerCommand("Set Shooter Auto", new InstantCommand(() -> shooter.setShooterAuto(0.85)));
    NamedCommands.registerCommand("Reset Heading", new InstantCommand(drivetrain::zeroHeading, drivetrain));
    NamedCommands.registerCommand("Source Set Pivot Position", new InstantCommand(() -> shooter.setPivotIntendedPos(11.5)));
    NamedCommands.registerCommand("Set Shooter Outtake", new InstantCommand(() -> shooter.setOuttakeMode()));
    NamedCommands.registerCommand("Turn Forward", new RunCommand(() -> drivetrain.turnToHeading(0, new Translation2d())).until(() -> Math.abs(drivetrain.getHeading()) < 1));
    NamedCommands.registerCommand("Turn to Angle 5", new RunCommand(() -> drivetrain.turnToHeading(5, new Translation2d())).until(() -> Math.abs(drivetrain.getHeading() - 5) < 1));
    NamedCommands.registerCommand("Note Align", new RunCommand(() -> drivetrain.swerveDrive(
      0.5, 
      0, 
      -drivetrain.getNoteAlignSpeed(),
      false,
      new Translation2d(),
      true))
      .withTimeout(0.55));
  }

  public void setDefaultCommands(){
    drivetrain.setDefaultCommand(new SwerveDrive());
    intake.setDefaultCommand(new IntakeHold());
    shooter.setDefaultCommand(new ShooterHold());
    // climber.setDefaultCommand(new ClimberHold());
    ampBar.setDefaultCommand(new AmpBarHold());
  }

  private void configureAutoTab() {
    autoChooser = AutoBuilder.buildAutoChooser("A_4-6_Blue");
    autoTab.add("Auto Chooser", autoChooser).withWidget(BuiltInWidgets.kComboBoxChooser).withSize(2, 1).withPosition(4, 0);
  } 

  public void updateSimulationField(){
    if (swerveDriveSimulation != null) {
      SimulatedArena.getInstance().simulationPeriodic();

      Logger.recordOutput(
          "FieldSimulation/RobotPosition", swerveDriveSimulation.getSimulatedDriveTrainPose());

      final List<Pose3d> notes = SimulatedArena.getInstance().getGamePiecesByType("Note");
      if (notes != null) Logger.recordOutput("FieldSimulation/Notes", notes.toArray(Pose3d[]::new));
    }
  }
}
