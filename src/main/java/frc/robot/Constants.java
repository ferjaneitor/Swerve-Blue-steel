package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
//import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {


    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4); //(MODIFY)
        public static final double kDriveMotorGearRatio = 1 / 6.75; //(MODIFY)
        public static final double kSteeringMotorGearRatio = 1 / 21.4285714286; //(MODIFY)
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kSteeringEncoderRot2Rad = kSteeringMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kSteeringEncoderRPM2RadPerSec = kSteeringEncoderRot2Rad / 60;
        public static final double kPSteering = 0.4 ;
        public static final double kISteering = 0;
        public static final double KDSteering = 0;
        public static final String CANivore = "5133BlueSteel";

    }

    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(27); //(MODIFY)
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(27); //(MODIFY)
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));
        
        public static final Translation2d[] modulesOffSet = new Translation2d[] {
            new Translation2d(DriveConstants.kWheelBase / 2, -DriveConstants.kTrackWidth / 2),
            new Translation2d(DriveConstants.kWheelBase / 2, DriveConstants.kTrackWidth / 2),
            new Translation2d(-DriveConstants.kWheelBase / 2, -DriveConstants.kTrackWidth / 2),
            new Translation2d(-DriveConstants.kWheelBase / 2, DriveConstants.kTrackWidth / 2)
        };


        public static final int kFrontLeftDriveMotorPort = 1;
        public static final int kBackLeftDriveMotorPort = 3 ;
        public static final int kFrontRightDriveMotorPort = 5;
        public static final int kBackRightDriveMotorPort = 7 ;
    
        public static final int kFrontLeftSteeringMotorPort = 2;
        public static final int kBackLeftSteeringMotorPort = 4 ;
        public static final int kFrontRightSteeringMotorPort = 6 ;
        public static final int kBackRightSteeringMotorPort = 8 ;
    
        public static final boolean kFrontLeftSteeringEncoderReversed = false;
        public static final boolean kBackLeftSteeringEncoderReversed = false ;
        public static final boolean kFrontRightSteeringEncoderReversed = false ;
        public static final boolean kBackRightSteeringEncoderReversed = false ;
    
        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = false ;
        public static final boolean kFrontRightDriveEncoderReversed = false ;
        public static final boolean kBackRightDriveEncoderReversed = false ;
    
        public static final int kBackLeftDriveAbsoluteEncoderPort = 15 ;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 16 ;
        public static final int kBackRightDriveAbsoluteEncoderPort = 17 ;
        public static final int kFrontLeftDriveAbsoluteEncoderPort = 14 ;
    
        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = true;
    
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0.290039;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0.413574 ;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0.338135 ;
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -0.082031 ;
    

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond * 0.80; // Velocidad lineal maxima
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 2; // Velocidad angular maxima
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    public static final class OIConstants {
        public static final double kDeadband = 0.25;
    }

    public static final class FieldCosntants{
        
        public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    }

    public static final class VisionConstants{

        public static final String cameraName = "";

        public static final Transform3d kRobotToCam = new Transform3d( new Translation3d( 0 , 0 , 0 ), new Rotation3d( 0 , 0 , 0 ));

    }

    public static final class ElevatorConstants{

        public static final int ElevatorID = 10 ;

        public static final int PivotID = 9 ;

        public static final double ElevatorMaxSpeed = 0.5;

        public static final double PivotMaxSpeed = 0.3;

    }

}