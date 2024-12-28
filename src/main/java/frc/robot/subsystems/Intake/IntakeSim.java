package frc.robot.subsystems.Intake;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.crescendo2024.CrescendoNoteOnField;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class IntakeSim implements IntakeIO {
  private final IntakeSimulation intakeSim; 

  private final AbstractDriveTrainSimulation driveSim;

  private double intakeVoltage = 0.0;
  // This is an indefinite integral of the intake motor voltage since the note has been in the
  // intake.
  // This approximates the position of the note in the intake.
  private double intakeVoltageIntegralSinceNoteTaken = 0.0;

  public IntakeSim(AbstractDriveTrainSimulation driveSim) {
    intakeSim = new IntakeSimulation(
      "Note",
      driveSim,
      Units.inchesToMeters(27),
      IntakeSimulation.IntakeSide.BACK, 
      1
    );
    intakeSim.register();
    
    this.driveSim = driveSim;
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // gamePiecesInIntakeCount shows the amount of game pieces in the intake, we store this in the
    // inputs
    boolean intakeHasNote = intakeSim.getGamePiecesAmount() != 0;

    // if the intake voltage is higher than 2 volts, it is considered running
    if (intakeVoltage > 4) intakeSim.startIntake();
    // otherwise, it's stopped
    else intakeSim.stopIntake();

    if (intakeHasNote) {
      SimulatedArena.getInstance()
          .addGamePiece(
              new CrescendoNoteOnField(
                  driveSim
                      .getSimulatedDriveTrainPose()
                      .getTranslation()
                      .plus(
                          new Translation2d(0, 0.5))));
    }

    // if the there is note, we do an integral to the voltage to approximate the position of the
    // note in the intake
    if (intakeHasNote) intakeVoltageIntegralSinceNoteTaken += 0.02 * intakeVoltage;
    // if the note is gone, we clear the integral
    else intakeVoltageIntegralSinceNoteTaken = 0.0;

    // if the integral is negative, we get rid of the note
    if (intakeVoltageIntegralSinceNoteTaken < 0 && intakeSim.obtainGamePieceFromIntake())
      // splits the note out by adding it on field
      SimulatedArena.getInstance()
          .addGamePiece(
              new CrescendoNoteOnField(
                  driveSim
                      .getSimulatedDriveTrainPose()
                      .getTranslation()
                      .plus(
                          new Translation2d(-0.4, 0)
                              .rotateBy(driveSim.getSimulatedDriveTrainPose().getRotation()))));
    // if the intake have been running positive volts since the note touches the intake, it will
    // touch the fly wheels
    // else if (intakeVoltageIntegralSinceNoteTaken > 12 * 0.1
    //     && intakeSim.obtainGamePieceFromIntake())
    //   // launch the note by calling the shoot note call back
    //   passNoteToFlyWheelsCall.run();
  }

  @Override
  public boolean obtainGamePieceFromIntake() {
    return intakeSim.obtainGamePieceFromIntake();
  }

  @Override
  public boolean simHasNote() {
    return intakeSim.getGamePiecesAmount() != 0;
  }
}
