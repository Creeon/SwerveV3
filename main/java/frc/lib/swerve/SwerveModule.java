// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.robot.subsystems.DriveSubsystem;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import com.revrobotics.SparkPIDController;

public class SwerveModule {
  private static final double kWheelRadius = 0.0508;
  private static final int kEncoderResolution = 4096;

  private static final double kModuleMaxAngularVelocity = DriveSubsystem.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration =
      100 * Math.PI; // radians per second squared

  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turningEncoder;

  private final String ID;

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController = new PIDController(0.11525, 0, 0);

  // Gains are for example purposes only - must be determined for your own robot!
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
        4.0578,
          0,
          0.1499,
          new TrapezoidProfile.Constraints(
              99999, 99999));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0.11885, 2.4592, 0.10492);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(0.17645,0.42596,0.0063124);//Last

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotorChannel PWM output for the drive motor.
   * @param turningMotorChannel PWM output for the turning motor.
   * @param driveEncoderChannelA DIO input for the drive encoder channel A
   * @param driveEncoderChannelB DIO input for the drive encoder channel B
   * @param turningEncoderChannelA DIO input for the turning encoder channel A
   * @param turningEncoderChannelB DIO input for the turning encoder channel B
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int CANCoderID,
      double offset) {
    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);
    m_driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_turningMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_driveEncoder = m_driveMotor.getEncoder();
    m_turningEncoder = m_turningMotor.getEncoder();
    m_driveEncoder.setPosition(0);
    ID=Integer.toString(CANCoderID);
    CANcoder c = new CANcoder(CANCoderID);
    double pos = 360-360*c.getAbsolutePosition().getValueAsDouble()+offset;
    pos=pos%360;
    if(pos>180){
        pos=pos-360;
    }
    else if(pos<-180){
        pos=pos+360;
    }
    m_turningEncoder.setPosition(pos*Math.PI/180);
    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_driveEncoder.setPositionConversionFactor(2 * Math.PI * kWheelRadius / (6.25));
    m_driveEncoder.setVelocityConversionFactor(2 * Math.PI * kWheelRadius / (6.25)/60);

    // Set the distance (in this case, angle) in radians per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    m_turningEncoder.setPositionConversionFactor(2 * Math.PI / (150/7.0));
    m_turningEncoder.setVelocityConversionFactor(2 * Math.PI / (150/7.0)/60);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }
  
  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveEncoder.getVelocity(), new Rotation2d(m_turningEncoder.getPosition()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        //m_driveEncoder.getVelocity(), new Rotation2d(m_turningEncoder.getPosition()));
        m_driveEncoder.getPosition(), new Rotation2d(m_turningEncoder.getPosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    var encoderRotation = new Rotation2d(m_turningEncoder.getPosition());

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoderRotation);

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(m_driveEncoder.getVelocity(), state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(m_turningEncoder.getPosition(), state.angle.getRadians());

    final double turnFeedforward =
        m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);
    //SmartDashboard.putNumber("Speed", m_driveEncoder.getVelocity());
    m_driveMotor.setVoltage(driveOutput + driveFeedforward);
    //m_driveMotor.set(MathUtil.clamp(driveOutput,-1,1));
    m_turningMotor.setVoltage(turnOutput + turnFeedforward);
    SmartDashboard.putNumber(ID, driveOutput + driveFeedforward);
  }
  public CANSparkMax getDriveMotor(){
    return m_driveMotor;
  }
  public CANSparkMax getTurnMotor(){
    return  m_turningMotor;
  }
  public RelativeEncoder getDriveEncoder(){
    return m_driveEncoder;
  }
  public RelativeEncoder getTurnEncoder(){
    return m_turningEncoder;
  }
}