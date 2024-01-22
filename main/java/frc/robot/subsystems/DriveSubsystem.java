// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.AnalogGyro;

import edu.wpi.first.math.geometry.Pose2d;
import javax.swing.text.DefaultStyledDocument.ElementSpec;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.swerve.SwerveModule;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.PIDController;

public class DriveSubsystem extends SubsystemBase {
  public static final double kMaxSpeed = 5; // 3 meters per second
  public static final double kMaxAngularSpeed = 3.4*(kMaxSpeed/5)*Math.PI; // 1/2 rotation per second

  public double lastAngle = 0;

  private final Translation2d m_frontLeftLocation = new Translation2d(0.25, -0.25);
  private final Translation2d m_frontRightLocation = new Translation2d(0.25, 0.25);//0.762
  private final Translation2d m_backLeftLocation = new Translation2d(-0.25, -0.25);//9.5
  private final Translation2d m_backRightLocation = new Translation2d(-0.25, 0.25);
  //andrew is a poopy butt head- yashica
  private final SwerveModule m_frontLeft = new SwerveModule(5, 6, 4, 0);
  private final SwerveModule m_frontRight = new SwerveModule(2, 1, 1, 0);
  private final SwerveModule m_backLeft = new SwerveModule(8, 7, 3, 0);
  private final SwerveModule m_backRight = new SwerveModule(4, 3, 2, 0);
  

  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(9999);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(9999);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(9999);

  private final PIDController turnPID = new PIDController(0.1, 0, 0.0025);

  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);  

  private final double coef = 1.0/(1-0.05);

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          m_kinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          });
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    //m_gyro.calibrate();
    m_gyro.reset();
    turnPID.enableContinuousInput(-180, 180);
  }
  public void drive(
    double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds, boolean isTurning, double x, double y, double leftTrigger, double rightTrigger, int FOV) {
    
    xSpeed =
    m_xspeedLimiter.calculate(MathUtil.applyDeadband(xSpeed, 0.05))
        * kMaxSpeed;
    ySpeed =
    -m_yspeedLimiter.calculate(MathUtil.applyDeadband(ySpeed, 0.05))
        * kMaxSpeed;

    rot =
    -m_rotLimiter.calculate(MathUtil.applyDeadband(rot, 0.02))

        * kMaxAngularSpeed;

    /*if(rot==0){
      rot=turnPID.calculate(m_gyro.getYaw(),lastAngle);
    }
    else{
      lastAngle=m_gyro.getYaw();
    }*/
    if(Math.abs(x)>0.04){
          x=coef*(x-0.05);}
    if(Math.abs(y)>0.04){
          y=coef*(y-0.05);}
    if(rightTrigger>0.95){
      if(FOV!=-1){
        x=MathUtil.applyDeadband(x, 0.05);
        y=MathUtil.applyDeadband(y,0.05);
        
        


        double angle = Math.atan2(y,x);
        if(x==0 && y==0){
          angle=-90*Math.PI/180;
        }
        angle = normalizeAngle((angle)*180/Math.PI+90);
        //rot=-turnPID.calculate(m_gyro.getYaw(),angle);
      }
      else{
        //rot=-turnPID.calculate(m_gyro.getYaw(),(double) FOV);
      }
      rot = turnPID.calculate(LimelightSubsystem.getXCoordinate(),0);
      SmartDashboard.putNumber("ROT", rot);
      SmartDashboard.putNumber("X", LimelightSubsystem.getXCoordinate());
    }
    SmartDashboard.putNumber("RAAHA", FOV);
    if(FOV!=-1){
      double e = FOV;
      rot=-turnPID.calculate(m_gyro.getYaw(),(double) e);
    }
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, Rotation2d.fromDegrees(m_gyro.getYaw()))
                    : new ChassisSpeeds(xSpeed, ySpeed, rot)
                );
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    /*if(leftTrigger>0.95){
    swerveModuleStates[0].speedMetersPerSecond=0;
    swerveModuleStates[1].speedMetersPerSecond=0;
    swerveModuleStates[2].speedMetersPerSecond=0;
    swerveModuleStates[3].speedMetersPerSecond=0;}*/
    if(leftTrigger>0.95){
    swerveModuleStates[0].angle=Rotation2d.fromDegrees(-45);
    swerveModuleStates[1].angle=Rotation2d.fromDegrees(45);
    swerveModuleStates[2].angle=Rotation2d.fromDegrees(45);
    swerveModuleStates[3].angle=Rotation2d.fromDegrees(-45);
    swerveModuleStates[0].speedMetersPerSecond=0;
    swerveModuleStates[1].speedMetersPerSecond=0;
    swerveModuleStates[2].speedMetersPerSecond=0;
    swerveModuleStates[3].speedMetersPerSecond=0;}
    
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });
  }
  public AHRS getGyro(){
    return m_gyro;
  }
  
  public double normalizeAngle(double angle){
    if(Math.abs(angle)<=180){
      return angle;
    }
    else{
      if(angle>180){
        return angle-360;
      }
      else{
        return angle+360;
      }
    }
  }
  public Pose2d getPose2d(){
    return m_odometry.getPoseMeters();
  }
  public PIDController getHeadingController(){
    return turnPID;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run\
    SmartDashboard.putNumber("Drive1 Velocity", m_frontRight.getDriveEncoder().getVelocity());
    SmartDashboard.putNumber("Drive2 Velocity", m_backRight.getDriveEncoder().getVelocity());
    SmartDashboard.putNumber("Drive3 Velocity", m_frontLeft.getDriveEncoder().getVelocity());
    SmartDashboard.putNumber("Drive4 Velocity", m_backLeft.getDriveEncoder().getVelocity());
    SmartDashboard.putNumber("Drive Pos 1", m_frontRight.getTurnEncoder().getPosition());
    SmartDashboard.putNumber("Drive Pos 2", m_backRight.getTurnEncoder().getPosition());
    SmartDashboard.putNumber("Drive Pos 3", m_frontLeft.getTurnEncoder().getPosition());
    SmartDashboard.putNumber("Drive Pos 4", m_backLeft.getTurnEncoder().getPosition());
    SmartDashboard.putNumber("Angle", m_gyro.getAngle());
    SmartDashboard.putNumber("Yaw", m_gyro.getYaw());
    SmartDashboard.putNumber("XPos", m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("YPos",m_odometry.getPoseMeters().getY());
    updateOdometry();
  }
}
