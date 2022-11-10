// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.k;

public class Chassis extends SubsystemBase {
  private final SwerveModule m_LF;
  private final SwerveModule m_RF;
  private final SwerveModule m_B;

  private final AHRS m_gyro;
  /** Creates a new Chassis. */
  public Chassis() {
    m_LF = new SwerveModule(k.SWERVE.LFData);
    m_RF = new SwerveModule(k.SWERVE.RFData);
    m_B = new SwerveModule(k.SWERVE.BData);

    m_gyro = new AHRS(Port.kMXP);
    m_gyro.calibrate();

    SmartDashboard.putData(m_B);
    SmartDashboard.putData(m_LF);
    SmartDashboard.putData(m_RF);


  
  }

  public void drive(double _xSpeed, double _ySpeed, double _rot){
    double xSpeed = _ySpeed * k.SWERVE.kMaxVel_MPS;
    double ySpeed = -_xSpeed * k.SWERVE.kMaxVel_MPS;
    double rot = _rot * k.CHASSIS.kRobotRadiansPerSec;

    SwerveModuleState[] swerveModuleStates = k.CHASSIS.kDrivetrainKinematics.toSwerveModuleStates(k.CHASSIS.IsFieldRelative
    ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
    : new ChassisSpeeds(xSpeed,ySpeed,rot));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, k.SWERVE.kMaxVel_MPS);

    m_LF.setDesiredState(swerveModuleStates[0]);
    m_RF.setDesiredState(swerveModuleStates[1]);
    m_B.setDesiredState(swerveModuleStates[2]);
  }
  public double getRobotAngle(){
    return -m_gyro.getAngle();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  @Override
  public void initSendable(SendableBuilder builder){
    super.initSendable(builder);
    builder.addDoubleProperty("RobotAngle_Deg", this::getRobotAngle, null);
  }
}
