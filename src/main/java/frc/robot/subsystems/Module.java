package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.util.MotorFactory;
import lib.ctre_shims.TalonEncoder;

public abstract class Module {
  
  public static Module create(int driveMotorID, int steerMotorID, int encoderID, double steerOffset) {
    if (Robot.isReal()) {
      return new ModuleReal(driveMotorID, steerMotorID, encoderID, steerOffset);
    } else {
      // return new ModuleSim(driveMotorID, steerMotorID, encoderID, steerOffset);
    }
    return null;
  }

  private final WPI_TalonFX m_driveMotor;
  private final WPI_TalonFX m_steerMotor;

  private final TalonEncoder m_driveEncoder;
  private final WPI_CANCoder m_encoder;

  private final PIDController m_drivePIDController = new PIDController(Constants.drive.kDriveP,
            Constants.drive.kDriveI, Constants.drive.kDriveD);

  private final ProfiledPIDController m_steerPIDController = new ProfiledPIDController(
          Constants.drive.kSteerP,
          Constants.drive.kSteerI,
          Constants.drive.kSteerD,
          new TrapezoidProfile.Constraints(Constants.drive.kMaxAngularSpeed, 2 * Math.PI));

  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(Constants.drive.kDriveKS,
          Constants.drive.kDriveKV);
  private final SimpleMotorFeedforward m_steerFeedforward = new SimpleMotorFeedforward(Constants.drive.kSteerKS,
          Constants.drive.kSteerKV);


  public Module(int driveMotorID, int steerMotorID, int encoderID, double steerOffset) {
    m_driveMotor = MotorFactory.createTalonFX(driveMotorID, Constants.kCanivoreCAN, 40, 80, 1, NeutralMode.Brake);
    m_steerMotor = MotorFactory.createTalonFX(steerMotorID, Constants.kCanivoreCAN, 30, 60, 1, NeutralMode.Brake);

    m_driveMotor.setNeutralMode(NeutralMode.Brake);
    m_steerMotor.setNeutralMode(NeutralMode.Brake);

    m_driveEncoder = new TalonEncoder(m_driveMotor);
    m_encoder = new WPI_CANCoder(encoderID, Constants.kCanivoreCAN);
  }


  public WPI_TalonFX getDriveMotor() {
    return m_driveMotor;
  }

  public WPI_TalonFX getSteerMotor() {
    return m_steerMotor;
  }

  public TalonEncoder getDriveEncoder() {
    return m_driveEncoder;
  }

  public WPI_CANCoder getEncoder() {
    return m_encoder;
  }

  public PIDController getDrivePIDController() {
    return m_drivePIDController;
  }

  public ProfiledPIDController getSteerPIDController() {
    return m_steerPIDController;
  }

  public SimpleMotorFeedforward getDriveFeedforward() {
    return m_driveFeedforward;
  }

  public SimpleMotorFeedforward getSteerFeedforward() {
    return m_steerFeedforward;
  }


  public abstract double getAngle();
  public abstract double getTurnOutput();
  public abstract double getDriveVelocity();
  public abstract double getCharacterizationVelocity();
  public abstract void setDesiredState(SwerveModuleState state);

  public abstract void characterize(double volts);
  
}
