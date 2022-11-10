// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.InvertType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.subsystems.SwerveData;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class k {
    public static class SWERVE{
       public static final double Steer_kP = 0.15;
       public static final double Steer_kD = 0.0;
       public static final double Steer_kI = 0.0;

       public static final double kMaxDriveEncVel_UnitsPer100ms = 21777.0; // Measured from Encoder in native units
       public static final double kMaxVel_MPS = 4.324; // Calculated from gear ratios and wheel circumference
       public static final double kDriveEncVelRatio = kMaxVel_MPS / (kMaxDriveEncVel_UnitsPer100ms * 10);

       public static final double kSteerMotEncoderCountsPerRev = 2048.0;
       public static final double kSteerRatio = 15.4286;
       public static final double kSteerMotCntsPerWheelDeg = (kSteerMotEncoderCountsPerRev * kSteerRatio) / 360;  // Cnts/Deg
       public static final double kSteerMotCountsPerWheelRadian = (kSteerMotEncoderCountsPerRev / (2 * Math.PI)) * kSteerRatio; // Cnts/Rad

       public static SwerveData LFData = new SwerveData("LF", 0, InvertType.None, 0, InvertType.None, 0, 0.0);
       public static SwerveData RFData = new SwerveData("RF", 0, InvertType.None, 0, InvertType.None, 0, 0.0);
       public static SwerveData BData = new SwerveData("B", 0, InvertType.None, 0, InvertType.None, 0, 0.0);


    }
    public static final class CHASSIS{
        public static final double kTrackWidth = 0.474;
        public static final double kWheelBase = 0.474;
        public static final SwerveDriveKinematics kDrivetrainKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2), // Left Front
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // Right Front
            new Translation2d(-kTrackWidth / 2, 0)  // Back
        );
        public static final double kRobotRadiansPerSec = 12.9;
        public static boolean IsFieldRelative = true;

    }
}
