package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.*;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.crescendo2024.CrescendoNoteOnField;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.TransportConstants;

public class IntakeSim implements IntakeIO {
  private final IntakeSimulation intakeSim; 

  private final AbstractDriveTrainSimulation driveSim;

  private double intakeVoltage = 0.0, transportVoltage = 0.0, notePosition = -1;

  public IntakeSim(AbstractDriveTrainSimulation driveSim) {
    intakeSim = IntakeSimulation.InTheFrameIntake(
      "Note", 
      driveSim, 
      Inches.of(27), 
      IntakeSimulation.IntakeSide.FRONT, 
      1);

    intakeSim.register();
    
    this.driveSim = driveSim;

    preloadNote();
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

    notePosition = inputs.hasNote ? Math.max(0, Math.min(notePosition, 1)) : -1;

    inputs.notePosition = this.notePosition;

    // if the intake voltage is higher than 2 volts, it is considered running
    if (intakeVoltage > 4) intakeSim.startIntake();
    // otherwise, it's stopped
    else intakeSim.stopIntake();
  }

  @Override
  public void setIntake(double speed) {
    intakeVoltage = speed * 12;
  }

  @Override
  public void setTransport(double speed) {
    transportVoltage = speed * 12;
    if (speed > TransportConstants.TRANSPORT_HOLD_SPEED) { // shoot
      notePosition += speed * 0.1;

      // note: do not call Shooter.getInstance() before a Shooter instance is created
      // if (notePosition >= 0.9 && intakeSim.obtainGamePieceFromIntake()) {
      //   try {
      //     Shooter.getInstance().shootNote();
      //   } catch (IllegalStateException e) {
      //     e.printStackTrace();
      //   }
      // }
    } else if (notePosition > -1) { // outtake/hold
      // holds at pos 0.5, will decrease below 0.5 if negative
      notePosition = Math.min(notePosition + (speed * 0.1), 0.5); 
      
      // splits the note out by adding it on field
      if (speed < 0 && notePosition <= 0.1 && intakeSim.obtainGamePieceFromIntake()) {
        SimulatedArena.getInstance()
            .addGamePiece(
                new CrescendoNoteOnField(
                    driveSim
                        .getSimulatedDriveTrainPose()
                        .getTranslation()
                        .plus(
                            new Translation2d(0.6, 0)
                                .rotateBy(driveSim.getSimulatedDriveTrainPose().getRotation()))));
      }
    }
  }

  @Override
  public boolean obtainGamePieceFromIntake() {
    return intakeSim.obtainGamePieceFromIntake();
  }

  private void preloadNote() {
    intakeSim.startIntake();

    SimulatedArena.getInstance()
            .addGamePiece(
                new CrescendoNoteOnField(
                    driveSim
                        .getSimulatedDriveTrainPose()
                        .getTranslation()
                        .plus(
                            new Translation2d(0.1, 0)
                                .rotateBy(driveSim.getSimulatedDriveTrainPose().getRotation()))));
  }
}
