package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.*;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.crescendo2024.CrescendoNoteOnField;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.Shooter.Shooter;

public class IntakeSim implements IntakeIO {
  private final IntakeSimulation intakeSim; 

  private final AbstractDriveTrainSimulation driveSim;

  private double intakeVoltage = 0.0, transportVoltage = 0.0;

  public IntakeSim(AbstractDriveTrainSimulation driveSim) {
    intakeSim = IntakeSimulation.InTheFrameIntake(
      "Note", 
      driveSim, 
      Inches.of(27), 
      IntakeSimulation.IntakeSide.FRONT, 
      1);

    intakeSim.register();
    
    this.driveSim = driveSim;
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeVolts = intakeVoltage;
    inputs.transportVolts = transportVoltage;

    // TODO: note detection sim(?)
    inputs.hasTarget = false;

    // gamePiecesInIntakeCount shows the amount of game pieces in the intake, we store this in the
    // inputs
    inputs.hasNote = intakeSim.getGamePiecesAmount() != 0;

    // if the intake voltage is higher than 2 volts, it is considered running
    if (intakeVoltage > 4) intakeSim.startIntake();
    // otherwise, it's stopped
    else intakeSim.stopIntake();

    // TODO: visualize note in robot
  }

  @Override
  public void setIntake(double speed) {
    intakeVoltage = speed * 12;
  }

  @Override
  public void setTransport(double speed) {
    if (speed > 0.35 /* && intakeSim.obtainGamePieceFromIntake() */) { 
      // note: do not call Shooter.getInstance() before a Shooter instance is created
      Shooter.getInstance().shootNote(); 
    } else if (speed < -0.35 /* && intakeSim.obtainGamePieceFromIntake() */) {
      // splits the note out by adding it on field
      SimulatedArena.getInstance()
          .addGamePiece(
              new CrescendoNoteOnField(
                  driveSim
                      .getSimulatedDriveTrainPose()
                      .getTranslation()
                      .plus(
                          new Translation2d(0.4, 0)
                              .rotateBy(driveSim.getSimulatedDriveTrainPose().getRotation()))));
    }
  }
}
