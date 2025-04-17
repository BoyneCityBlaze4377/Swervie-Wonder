package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
  private final String m_name;

  private final SparkMax m_driveMotor;
  private final SparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder, m_turningEncoder;

  private final SparkMaxConfig m_driveConfig, m_turnConfig;

  private final AnalogInput absoluteEncoder;

  private double AnalogEncoderOffset, turningFactor;
  private final boolean driveInverted, turnReversed, absReversed;

  private Rotation2d lastAngle;

  private final SlewRateLimiter filter = new SlewRateLimiter(ModuleConstants.moduleDriveSlewRate);
  
  private final PIDController turningController;

  private final GenericEntry desiredStateSender, wheelAngle;

  /**
   * Constructs a SwerveModule.
   * 
   * @param name                   The name of the module.
   * @param driveMotorChannel      The channel of the drive motor.
   * @param turningMotorChannel    The channel of the turning motor.
   * @param turningEncoderChannel  The channels of the turning encoder.
   * @param driveMotorReversed     Whether the drive encoder is reversed.
   * @param turningMotorReversed   Whether the turning encoder is reversed.
   * @param encoderOffset          The offset, in degrees, of the absolute encoder.
   * @param absoluteEncoderReversed  Whether the absolute encoder is reversed (negative).
   */
  public SwerveModule(String name, int driveMotorChannel, int turningMotorChannel, 
                      int turningEncoderChannel, boolean driveMotorReversed, boolean turningMotorReversed, 
                      double encoderOffset, boolean absoluteEncoderReversed) {
    /** Name */
    m_name = name;
    
    /** Motors */
    m_driveMotor = new SparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new SparkMax(turningMotorChannel, MotorType.kBrushless);

    m_driveConfig = new SparkMaxConfig();
    m_turnConfig = new SparkMaxConfig();

    m_driveEncoder = m_driveMotor.getEncoder();
    m_turningEncoder = m_turningMotor.getEncoder();
    
    driveInverted = driveMotorReversed;
    turnReversed = turningMotorReversed;
    
    configAngleMotorDefault();
    configDriveMotorDefault();
    
    /** PIDController */
    turningController = new PIDController(ModuleConstants.angleKP, 
                                          ModuleConstants.angleKD, 
                                          ModuleConstants.angleKI);
    turningController.setTolerance(ModuleConstants.kTolerance);
    turningController.enableContinuousInput(-180, 180);

    /** Absolute Encoder */
    absoluteEncoder = new AnalogInput(turningEncoderChannel);
    AnalogEncoderOffset = encoderOffset;
    absReversed = absoluteEncoderReversed;

    resetEncoders();

    /** DashBoard Initialization */
    wheelAngle = IOConstants.DiagnosticTab.add(m_name + "'s angle", getAbsoluteEncoder()).getEntry();
    desiredStateSender = IOConstants.DiagnosticTab.add(m_name + "'s desired state", getState().toString()).getEntry();

    // LastAngle
    lastAngle = getState().angle;
  }

  /** @return The angle, in degrees, of the module */
  private Rotation2d getAngle() {
    return Rotation2d.fromDegrees(getAbsoluteEncoder());
  }

  /** @return The current state of the module. */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveEncoder.getVelocity(), getAngle());
  }

  /** @return The current position of the module. */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(m_driveEncoder.getPosition(), new Rotation2d(Units.degreesToRadians(getAbsoluteEncoder())));
  }

  /** @deprecated Use getAbsoluteEncoder() instead
   * @return The position of the relatoce encoder of the turning motor
   */
  public double getEncoderPos() {
    return m_turningEncoder.getPosition();
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState, boolean isNeutral) {
    SwerveModuleState state = desiredState;
    state.optimize(lastAngle);

    final double driveOutput = state.speedMetersPerSecond / DriveConstants.maxSpeedMetersPerSecond;

    m_driveMotor.set(filter.calculate(driveOutput));
    setAngle(state, isNeutral);

    desiredStateSender.setString(desiredState.toString());
  }

  /**
   * Sets the angle of the module's turn motor.
   * 
   * @param desiredState The desired state of the module.
   */
  public void setAngle(SwerveModuleState desiredState, boolean isNeutral) {
    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    Rotation2d angle = 
        (Math.abs(desiredState.speedMetersPerSecond) <= (DriveConstants.maxSpeedMetersPerSecond * .01)) ? lastAngle : desiredState.angle;
    
    turningFactor = MathUtil.clamp(turningController.calculate(getAbsoluteEncoder(), angle.getDegrees()), 
                                   -ModuleConstants.kMaxOutput, ModuleConstants.kMaxOutput);
    
    m_turningMotor.set(isNeutral || turningController.atSetpoint() ? 0 : -turningFactor);
    lastAngle = angle;
  }

  /** Resets all of the SwerveModule's encoders. */
  public void resetEncoders() {
    m_driveEncoder.setPosition(0);
    m_turningEncoder.setPosition(getAbsoluteEncoder());
  }

  /** @return The drive motor of the module. */
  public SparkMax getDriveMotor() {
    return m_driveMotor;
  }

  /** @return The turn motor of the module. */
  public SparkMax getTurnMotor() {
    return m_turningMotor;
  }

  /** Stops the module's drive motor from moving. */
  public void stop() {
    m_driveMotor.set(0);
  }

  /** @return The angle, in degrees, of the module. */
  public double getAbsoluteEncoder() {
    double angle = absoluteEncoder.getAverageVoltage() / RobotController.getVoltage5V();
    angle *= 360;
    angle -= AnalogEncoderOffset;
    angle = MathUtil.inputModulus(angle, -180, 180);
    return (absReversed ? -1 : 1) * angle;
  }

  /** Posts module values to ShuffleBoard. */
  public void update() {
    wheelAngle.setDouble(getAbsoluteEncoder());
  }

  /** @return The distance the drive motor has moved. */
  public double getDistance() {
    return m_driveEncoder.getPosition();
  }

  /** Sets the module's drive motor's idle mode to brake. */
  public void brake() {
    m_driveConfig.idleMode(IdleMode.kBrake);
    configDriveMotor();
  }

  /** Sets the module's drive motor's idle mode to coast. */
  public void coast() {
    m_driveConfig.idleMode(IdleMode.kCoast);
    configDriveMotor();
  }

  /** Sets the SparkMaxConfig of the turning motor to m_turnConfig */
  public void configAngleMotor() {
    m_turningMotor.configure(m_turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /** Sets the SparkMaxConfig of the drive motor to m_driveConfig */
  public void configDriveMotor() {
    m_driveMotor.configure(m_driveConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /** Sets the default configuration of the angle motor. */
  private void configAngleMotorDefault() {
    m_turnConfig.idleMode(ModuleConstants.angleNeutralMode);
    m_turnConfig.inverted(turnReversed);
    m_turnConfig.smartCurrentLimit(ModuleConstants.angleContinuousCurrentLimit);
    m_turnConfig.voltageCompensation(ModuleConstants.voltageComp);

    configAngleMotor();
    Timer.delay(1);
  }

  /** Sets the default configuration of the drive motor. */
  private void configDriveMotorDefault() {
    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder resolution.
    m_driveConfig.encoder.velocityConversionFactor(ModuleConstants.driveMotorConversionFactor/60);
    m_driveConfig.encoder.positionConversionFactor(ModuleConstants.driveMotorConversionFactor);

    m_driveConfig.idleMode(IdleMode.kBrake);
    m_driveConfig.inverted(driveInverted);

    Timer.delay(1);
    configDriveMotor();
  }

  /** 
   * Sets the state of the module, ONLY USE FOR LOCKED MODE.
   * 
   * @param lockedState The state with which to lock the module 
   */
  public void setLockedState(SwerveModuleState lockedState) {
    lockedState.optimize(lastAngle);

    m_driveMotor.set(0);
    turningFactor = turningController.calculate(getAbsoluteEncoder(), lockedState.angle.getDegrees());

    turningFactor = MathUtil.clamp(turningFactor, -ModuleConstants.kMaxOutput, ModuleConstants.kMaxOutput);

    m_turningMotor.set(turningController.atSetpoint() ? 0 : -turningFactor);
    lastAngle = lockedState.angle;
  }

  /** Stops the module's angle motor from moving. */
  public void stopTurn() {
    m_turningMotor.set(0);
  }

  /** @return The current IdleMode of the Module. */
  public IdleMode getIdleMode() {
    return m_driveMotor.configAccessor.getIdleMode();
  }

  /** 
   * Sets the module's drive motor's IdleMode.
   * 
   * @param mode The IdleMode to set the module's drive motor to.
   */
  public void setIdleMode(IdleMode mode) {
    m_driveConfig.idleMode(mode);
    configDriveMotor();
  }
}