package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.WolfSparkMax;
import frc.robot.utils.Constants.SwerveConstants;

public class SwerveModule extends SubsystemBase {
  private WolfSparkMax driveMotor;
  private WolfSparkMax turnMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder turnEncoder;

  private PIDController turnPIDController;
  private CANcoder absoluteEncoder;

  private boolean absoluteEncoderReversed;
  private double absoluteEncoderOffset;

  private int driveID = 0;
  private int turnID = 0;

  private Rotation2d lastAngle;

  /** Creates a new SwerveModule. */
  public SwerveModule(int driveMotorId, int turnMotorId, boolean driveMotorReversed, boolean turnMotorReversed,
      int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
    this.absoluteEncoderOffset = absoluteEncoderOffset;
    this.absoluteEncoderReversed = absoluteEncoderReversed;

    driveID = driveMotorId;
    absoluteEncoder = new CANcoder(absoluteEncoderId);

    turnID = turnMotorId;

    driveMotor = new WolfSparkMax(driveMotorId, MotorType.kBrushless, IdleMode.kCoast, 45, driveMotorReversed);
    turnMotor = new WolfSparkMax(turnMotorId, MotorType.kBrushless, IdleMode.kCoast, 25, turnMotorReversed);

    driveEncoder = driveMotor.getEncoder();
    turnEncoder = turnMotor.getEncoder();

    turnPIDController = new PIDController(SwerveConstants.KP_TURNING, 0, 0);
    turnPIDController.enableContinuousInput(-Math.PI, Math.PI);

    resetEncoders();
    lastAngle = getState().angle;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Drive Motor: " + driveID, getDriveMotorPosition());
    SmartDashboard.putNumber("Turn Motor: " + turnID, getTurnMotorPosition());
    SmartDashboard.putNumber("Abs Wheel Angle (deg) - Motor: " + driveID,
        absoluteEncoder.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("Vels - Motor: " + driveID, getDriveMotorVelocity());
  }

  public void setBrake(boolean brake) {
    SparkMaxConfig config = new SparkMaxConfig();
    SparkMaxConfig config1 = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    config1.idleMode(IdleMode.kCoast);

    if (brake) {
      driveMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    } else {
      driveMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    turnMotor.configure(config1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public double getDriveMotorPosition() {
    return driveEncoder.getPosition() * SwerveConstants.DRIVE_MOTOR_PCONVERSION;
  }

  public double getDriveMotorVelocity() {
    return driveEncoder.getVelocity() * SwerveConstants.DRIVE_MOTOR_VCONVERSION;
  }

  public double getTurnMotorPosition() {
    return turnEncoder.getPosition() * SwerveConstants.TURN_MOTOR_PCONVERSION;
  }

  public double getTurnMotorVelocity() {
    return turnEncoder.getVelocity() * SwerveConstants.TURN_MOTOR_VCONVERSION;
  }

  public double getAbsoluteEncoderAngle() {
    double angle = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
    angle -= absoluteEncoderOffset;
    angle *= (2 * Math.PI);
    return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
  }

  public void resetEncoders() {
    driveEncoder.setPosition(0);
    turnEncoder.setPosition(getAbsoluteEncoderAngle() / SwerveConstants.TURN_MOTOR_PCONVERSION);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveMotorVelocity(), new Rotation2d(getTurnMotorPosition()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDriveMotorPosition(), new Rotation2d(getTurnMotorPosition()));
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    desiredState.optimize(getState().angle);

    setAngle(desiredState);
    setSpeed(desiredState);
    SmartDashboard.putString("Swerve [" + driveMotor.getDeviceId() + "] State", getState().toString());
    SmartDashboard.putNumber("Abs Angle " + driveMotor.getDeviceId(), getAbsoluteEncoderAngle());
  }

  public void setSpeed(SwerveModuleState desiredState) {
    driveMotor.set(desiredState.speedMetersPerSecond / SwerveConstants.DRIVETRAIN_MAX_SPEED);
  }

  public void setAngle(SwerveModuleState desiredState) {
    Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConstants.DRIVETRAIN_MAX_SPEED * 0.01))
        ? lastAngle
        : desiredState.angle; // Prevent rotating module if speed is less then 1%. Prevents Jittering.

    turnMotor.set(turnPIDController.calculate(getTurnMotorPosition(), desiredState.angle.getRadians()));
    lastAngle = angle;
  }

  public void stop() {
    driveMotor.set(0);
    turnMotor.set(0);
  }
}