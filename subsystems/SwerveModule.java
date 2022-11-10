// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.k;


/** Add your docs here. */
public class SwerveModule implements Sendable{
    private TalonFX m_steerMotor;
    private TalonFX m_driveMotor;
    private CANCoder m_steerEncoder;
    private SwerveData m_configData;
    private PIDController m_pid;
    
    public SwerveModule(SwerveData _data) {
        m_configData = _data;
        m_steerMotor = new TalonFX(m_configData.steerCANID);
        ErrorCode errorD = m_steerMotor.configFactoryDefault(250);
        if(errorD != ErrorCode.OK) {System.out.println(m_configData.toString() + "SwerveModule Error = " + errorD.toString());}

        m_steerMotor.setInverted(m_configData.steerInvert);
        m_steerMotor.setNeutralMode(NeutralMode.Brake);
        m_steerMotor.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Signed_PlusMinus180);

        m_pid = new PIDController(k.SWERVE.Steer_kP, k.SWERVE.Steer_kI, k.SWERVE.Steer_kD);
        m_pid.enableContinuousInput(0, 180);
        m_pid.setTolerance(0.1);

    }
    public void setDesiredState(SwerveModuleState _state) {
        setDesiredState(_state, true, false,false );
    }
    public void setDesiredState(SwerveModuleState _state, boolean _isOptimized, boolean _disableDrive, boolean _disableSteer){
        SwerveModuleState sms = _isOptimized ? SwerveModuleState.optimize(_state, getRotation2d()) : _state;
        
        double steerAng = sms.angle.getRadians();
        double out = m_pid.calculate(steerAng);

        m_steerMotor.set(TalonFXControlMode.PercentOutput, _disableSteer ? 0.0 : out);

        m_driveMotor.set(TalonFXControlMode.PercentOutput, _disableDrive ? 0.0 : sms.speedMetersPerSecond / k.SWERVE.kMaxVel_MPS);

    }
    public double getSteerMotAngInRad() {
        double motCnts = m_steerMotor.getSelectedSensorPosition();
        motCnts %= 2047;
        return motCnts /k.SWERVE.kSteerMotCountsPerWheelRadian;
    }
    public Rotation2d getRotation2d(){
        return new Rotation2d(getSteerMotAngInRad());
    }
    public double getSteerEncAng_Deg(){
        return m_steerEncoder.getAbsolutePosition();
    }
    public double getDriveMotorCnts(){
        return m_driveMotor.getSelectedSensorPosition();
    }
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SM");
        builder.addDoubleProperty("." + m_configData.name + "_StrAbsAngRaw", this::getSteerEncAng_Deg, null);
        builder.addDoubleProperty("." + m_configData.name + "_StrAbsAng", () -> getSteerEncAng_Deg() - m_configData.angleOffset_Deg, null);
        builder.addDoubleProperty("."+ m_configData.name + "_RelAngDeg", () -> Units.radiansToDegrees(getSteerMotAngInRad()), null);
        builder.addDoubleProperty("."+ m_configData.name + "_DrvMotCnts", () -> getDriveMotorCnts(), null);
        
    }
}
