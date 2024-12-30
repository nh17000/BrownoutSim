// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class IOConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OP_CONTROLLER_PORT = 1;
  }

  public static final class SwerveConstants{
    //Drivetrain motor/encoder IDs
    public static final int LEFT_FRONT_DRIVE_ID = 1;
    public static final int RIGHT_FRONT_DRIVE_ID = 2;
    public static final int LEFT_BACK_DRIVE_ID = 3;
    public static final int RIGHT_BACK_DRIVE_ID = 4;
    
    public static final int LEFT_FRONT_TURN_ID = 5;
    public static final int RIGHT_FRONT_TURN_ID = 6;
    public static final int LEFT_BACK_TURN_ID = 7;
    public static final int RIGHT_BACK_TURN_ID = 8;
    
    public static final int LEFT_FRONT_CANCODER_ID = 11;
    public static final int RIGHT_FRONT_CANCODER_ID = 12;
    public static final int LEFT_BACK_CANCODER_ID = 13;
    public static final int RIGHT_BACK_CANCODER_ID = 14;

    public static final int PIGEON_ID = 15;

    //Drivetrain characteristics
    public static final double LEFT_FRONT_OFFSET = -0.03564;
    public static final double RIGHT_FRONT_OFFSET = 0.43579;
    public static final double LEFT_BACK_OFFSET = 0.40991;
    public static final double RIGHT_BACK_OFFSET = -0.09301;

    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double DRIVE_MOTOR_GEAR_RATIO = 5.357; //SDS Mk4i L3+ (60:16 first stage)
    public static final double TURN_MOTOR_GEAR_RATIO = 150.0/7;
    public static final double DRIVE_MOTOR_PCONVERSION = WHEEL_DIAMETER * Math.PI / DRIVE_MOTOR_GEAR_RATIO;
    public static final double TURN_MOTOR_PCONVERSION = 2 * Math.PI / TURN_MOTOR_GEAR_RATIO;
    public static final double DRIVE_MOTOR_VCONVERSION = DRIVE_MOTOR_PCONVERSION;
    public static final double TURN_MOTOR_VCONVERSION = TURN_MOTOR_PCONVERSION;
    public static final double KP_TURNING = 0.5;

    public static final double DRIVETRAIN_MAX_SPEED = 6.62;
    public static final double DRIVETRAIN_MAX_ANGULAR_SPEED = 3.45 * Math.PI;

    //Swerve Kinematics
    public static final double TRACK_WIDTH = Units.inchesToMeters(21.75);
    public static final double WHEEL_BASE = Units.inchesToMeters(21.75);
    public static final double DRIVE_BASE_RADIUS = Math.sqrt(Math.pow(TRACK_WIDTH, 2) + Math.pow(WHEEL_BASE, 2)) / 2.0;

    public static final Translation2d[] MODULE_TRANSLATIONS = { 
      new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
      new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
      new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
      new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)
    };

    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(MODULE_TRANSLATIONS);

    //Teleop constraints
    public static final double TELE_DRIVE_MAX_SPEED = DRIVETRAIN_MAX_SPEED;
    public static final double TELE_DRIVE_MAX_ANGULAR_SPEED = DRIVETRAIN_MAX_ANGULAR_SPEED / 1.25;
    public static final double TELE_DRIVE_MAX_ACCELERATION = 3;
    public static final double TELE_DRIVE_MAX_ANGULAR_ACCELERATION = 3;

    //Auton constraints
    public static final double AUTO_kP_TRANSLATION = 4;
    public static final double AUTO_kP_ROTATION = 1.5;

    public static final double AUTO_DRIVE_MAX_SPEED = DRIVETRAIN_MAX_SPEED / 1.5;
    public static final double AUTO_DRIVE_MAX_ANGULAR_SPEED = DRIVETRAIN_MAX_ANGULAR_SPEED / 2.0;
    public static final double AUTO_DRIVE_MAX_ACCELERATION = 3;
    public static final double AUTO_DRIVE_MAX_ANGULAR_ACCELERATION = Math.PI;

    //https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose-estimators.html
    public static final Vector<N3> ODOMETRY_STD_DEV = VecBuilder.fill(0.1, 0.1, 0.1);

    public static final double kS_PERCENT = 0.035;
    public static final double kP_PERCENT = 0.006;

    public static final double ROBOT_MASS = 59.9;
    public static final double ROBOT_MOI = 6.883;
    public static final int DRIVE_CURRENT_LIMIT = 60;
    public static final int TURN_CURRENT_LIMIT = 20;

    public static final RobotConfig PATHPLANNER_CONFIG = new RobotConfig(
      ROBOT_MASS,
      ROBOT_MOI,
      new ModuleConfig(
        WHEEL_DIAMETER / 2,
        DRIVETRAIN_MAX_SPEED,
        COTS.WHEELS.VEX_GRIP_V2.cof,
        DCMotor.getKrakenX60(1).withReduction(DRIVE_MOTOR_GEAR_RATIO),
        DRIVE_CURRENT_LIMIT,
        1),
      MODULE_TRANSLATIONS);
    
    public static final DriveTrainSimulationConfig mapleSimConfig = new DriveTrainSimulationConfig(
        Kilograms.of(ROBOT_MASS), 
        Inches.of(27 + 3.25 * 2), // 27" x 27" base + 3.25" bumpers
        Inches.of(29 + 3.25 * 2), // 2" UTB intake
        Meters.of(TRACK_WIDTH), 
        Meters.of(TRACK_WIDTH), 
        COTS.ofMark4i(
            DCMotor.getKrakenX60(1), 
            DCMotor.getKrakenX60(1), 
            COTS.WHEELS.VEX_GRIP_V2.cof, 
            3), // L3 gear ratio, 150/7
        COTS.ofPigeon2());

    // public static final double DRIVE_FRICTION_VOLTAGE = 0.25;
    // public static final double TURN_FRICTION_VOLTAGE = 0.25;
    // public static final double TURN_INERTIA = 0.02;

    //   .withCustomModuleTranslations(MODULE_TRANSLATIONS)
    //   .withGyro(COTS.ofPigeon2())
    //   .withSwerveModule(() -> new SwerveModuleSimulation(
    //     DCMotor.getKrakenX60(1),
    //     DCMotor.getKrakenX60(1),
    //     DRIVE_MOTOR_GEAR_RATIO,
    //     TURN_MOTOR_GEAR_RATIO,
    //     Volts.of(DRIVE_FRICTION_VOLTAGE),
    //     Volts.of(TURN_FRICTION_VOLTAGE),
    //     Meters.of(WHEEL_DIAMETER / 2.0),
    //     KilogramSquareMeters.of(TURN_INERTIA),
    //     COTS.WHEELS.VEX_GRIP_V2.cof));

    public static final double ODOMETRY_FREQUENCY = 50.0;
    public static final double SIM_DRIVE_KS = 0.03;
    public static final double SIM_DRIVE_KV_ROT = 0.91035; // Same units as TunerConstants: (volt * secs) / rotation
    public static final double SIM_DRIVE_KV = 1.0 / Units.rotationsToRadians(1.0 / SIM_DRIVE_KV_ROT);
  }

  public static final class IntakeConstants{
    public static final int UTB_ROLLER_ID = 21;

    public static final double INTAKE_IN_SPEED = 0.8; 
    public static final double INTAKE_OUT_SPEED = -0.7; 

    public static final double RUMBLE_AT_CURRENT = 40.0;
  }


  public static final class ShooterConstants{
    public static final int LEFT_SHOOTER_ID = 31;
    public static final int RIGHT_SHOOTER_ID = 32;
    public static final int PIVOT_ID = 33;

    public static final double LEFT_SHOOTER_kP = 1.0; 
    public static final double LEFT_SHOOTER_kI = 0.001;
    public static final double LEFT_SHOOTER_kD = 0;

    public static final double RIGHT_SHOOTER_kP = 1.0;
    public static final double RIGHT_SHOOTER_kI = 0.001;
    public static final double RIGHT_SHOOTER_kD = 0;

    public static final double SHOOTER_MIN_OUTPUT = -1.0;
    public static final double SHOOTER_MAX_OUTPUT = 1.0;

    public static final double PIVOT_kP = 0.09; 
    public static final double PIVOT_kI = 0.00008;
    public static final double PIVOT_kD = 0;
    
    public static final double PIVOT_SIM_kP = PIVOT_kP * 100;

    public static final double PIVOT_MIN_OUTPUT = -0.85;
    public static final double PIVOT_MAX_OUTPUT = 0.85;

    public static final double AMP_PIVOT_POSITION = 27.3;
    public static final double PASSING_PIVOT_POSITION = 29.5;
    public static final double SPEAKER_PIVOT_POSITION = 32.4;
    public static final double TRAP_PIVOT_POSITION = 58;
    public static final double FLOOR_TO_SHOOTER = Units.inchesToMeters(7);
    public static final double PIVOT_OUTAKE_POSITION = 11.5;

    public static final double PASSING_VOLTAGE = 6.4;
    public static final double AMP_VOLTAGE = 4;
    public static final double SPEAKER_VOLTAGE = 5.5;
    public static final double TRAP_VOLTAGE = 4.3;
    public static final double OUTAKE_VOLTAGE = 2.3;

    public static final double PIVOT_ZEROING_SPEED = -0.075;

    public static final double LEFT_TO_RIGHT_VOLTAGE_OFFSET = 2.5;

    public static final Translation3d SHOOTER_TRANSLATION_ON_ROBOT = new Translation3d(0.108, 0, 0.154);
  }

  public static final class TransportConstants{
    public static final int TRANSPORT_ID = 22;

    public static final int IR_SENSOR_CHANNEL = 0;

    public static final double TRANSPORT_HOLD_SPEED = 0.35;
    public static final double TRANSPORT_OUT_SPEED = -0.4;
    public static final double TRANSPORT_SHOOT_SPEED = 1.0;
  }

  public static final class ClimberConstants{
    public static final int LEFT_CLIMBER_ID = 23;
    public static final int RIGHT_CLIMBER_ID = 24;

    public static final double MANTIS_ROT = 0;
    public static final double IDLE_ROT = 40;
    public static final double CLIMB_ROT = 90; //127.5

    public static final double CLIMBER_kP = 0.15;
    public static final double CLIMBER_kI = 0;
    public static final double CLIMBER_kD = 0;

    public static final double CLIMBER_MIN_OUTPUT = -1;
    public static final double CLIMBER_MAX_OUTPUT = 1;
  }

  public static final class AmpBarConstants{
    public static final int AMP_BAR_ID = 41;

    public static final double AMP_BAR_kP = 0.25;
    public static final double AMP_BAR_kI = 0;
    public static final double AMP_BAR_kD = 0;

    public static final double AMP_BAR_SIM_kP = AMP_BAR_kP * 7.5;

    public static final double AMP_BAR_MIN_OUTPUT = -0.5;
    public static final double AMP_BAR_MAX_OUTPUT = 0.5;

    public static final double STOWED_ROT = -19.1; // 135 degrees
    public static final double DEPLOYED_ROT = -1.0; // -60 degrees
    public static final double TRAP_ROT = -4.7;
    public static final double CLIMB_ROT = -3;
    public static final double DEFENSE_ROT = -6.7; // 0 degrees

    public static final Translation3d AMP_BAR_TRANSLATION_ON_ROBOT = new Translation3d(0.068, 0, 0.632);
  }

  public static final class FieldConstants{
    public static final double FIELD_LENGTH = 16.54175;
    public static final double FIELD_WIDTH = 8.21055;

    public static final double SPEAKER_HEIGHT = Units.inchesToMeters(80.515);

    public static final Pose2d INIT_SIM_POSE = new Pose2d(1.45, 7.33, new Rotation2d(0));
  }

  public static final class VisionConstants{
    public static final String SHOOTER_LL_NAME = "limelight-shooter";
    public static final String INTAKE_LL_NAME = "limelight-intake";

    public static final Vector<N3> LIMELIGHT_STD_DEV = VecBuilder.fill(.7, .7, .9999999);
    public static final Vector<N3> MEGATAG2_LIMELIGHT_STD_DEV = VecBuilder.fill(.7, .7, .9999999);

    
    public static final double AMBIGUITY_FILTER = 0.3;
    public static final double DISTANCE_FILTER = FieldConstants.FIELD_LENGTH / 2;
  }
}
