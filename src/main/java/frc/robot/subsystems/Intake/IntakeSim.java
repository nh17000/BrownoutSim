package frc.robot.subsystems.Intake;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.crescendo2024.CrescendoNoteOnField;

import edu.wpi.first.math.geometry.Translation2d;

public class IntakeSim implements IntakeIO {
  // TODO: update to 2025 wpilib to use newer versions of maple sim
  // private final IntakeSimulation intakeSim; 

  public IntakeSim(/*SimulatedArena arena, AbstractDriveTrainSimulation drivetrain*/) {
    // intakeSim = new IntakeSimulation( // create intake simulation with no extension
    //   arena,
    //   drivetrain,
    //   0.6, // check onshape for width of beta's intake
    //   IntakeSimulation.IntakeSide.BACK, 
    //   1
    // );
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // // gamePiecesInIntakeCount shows the amount of game pieces in the intake, we store this in the
    // // inputs
    // inputs.noteDetected = intakeSimulation.getGamePiecesAmount() != 0;

    // // if the intake voltage is higher than 2 volts, it is considered running
    // if (intakeVoltage > 4) intakeSimulation.startIntake();
    // // otherwise, it's stopped
    // else intakeSimulation.stopIntake();

    // // if the there is note, we do an integral to the voltage to approximate the position of the
    // // note in the intake
    // if (inputs.noteDetected) intakeVoltageIntegralSinceNoteTaken += 0.02 * intakeVoltage;
    // // if the note is gone, we clear the integral
    // else intakeVoltageIntegralSinceNoteTaken = 0.0;

    // // if the integral is negative, we get rid of the note
    // if (intakeVoltageIntegralSinceNoteTaken < 0 && intakeSimulation.obtainGamePieceFromIntake())
    //   // splits the note out by adding it on field
    //   SimulatedArena.getInstance()
    //       .addGamePiece(
    //           new CrescendoNoteOnField(
    //               driveTrain
    //                   .getSimulatedDriveTrainPose()
    //                   .getTranslation()
    //                   .plus(
    //                       new Translation2d(-0.4, 0)
    //                           .rotateBy(driveTrain.getSimulatedDriveTrainPose().getRotation()))));
    // // if the intake have been running positive volts since the note touches the intake, it will
    // // touch the fly wheels
    // else if (intakeVoltageIntegralSinceNoteTaken > 12 * 0.1
    //     && intakeSimulation.obtainGamePieceFromIntake())
    //   // launch the note by calling the shoot note call back
    //   passNoteToFlyWheelsCall.run();
  }
}
