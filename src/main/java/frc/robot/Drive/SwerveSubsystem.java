package frc.robot.Drive;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveSubsystem extends SubsystemBase {

    boolean isFieldOriented;
    
    private final SwerveModule FrontLeft = new SwerveModule(
        DriveConstants.kFrontLeftDriveMotorPort,
        DriveConstants.kFrontLeftSteeringMotorPort,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
        DriveConstants.kFrontLeftDriveEncoderReversed,
        DriveConstants.kFrontLeftSteeringEncoderReversed,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
        ModuleConstants.CANivore
    );
    
    private final SwerveModule FrontRight = new SwerveModule(
        DriveConstants.kFrontRightDriveMotorPort,
        DriveConstants.kFrontRightSteeringMotorPort,
        DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
        DriveConstants.kFrontRightDriveEncoderReversed,
        DriveConstants.kFrontRightSteeringEncoderReversed,
        DriveConstants.kFrontRightDriveAbsoluteEncoderReversed,
        DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
        ModuleConstants.CANivore
    );
    
    private final SwerveModule BackLeft = new SwerveModule(
        DriveConstants.kBackLeftDriveMotorPort,
        DriveConstants.kBackLeftSteeringMotorPort,
        DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
        DriveConstants.kBackLeftDriveEncoderReversed,
        DriveConstants.kBackLeftSteeringEncoderReversed,
        DriveConstants.kBackLeftDriveAbsoluteEncoderReversed,
        DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
        ModuleConstants.CANivore
    );
    
    private final SwerveModule BackRight = new SwerveModule(
        DriveConstants.kBackRightDriveMotorPort,
        DriveConstants.kBackRightSteeringMotorPort,
        DriveConstants.kBackRightDriveAbsoluteEncoderPort,
        DriveConstants.kBackRightDriveEncoderReversed,
        DriveConstants.kBackRightSteeringEncoderReversed,
        DriveConstants.kBackRightDriveAbsoluteEncoderReversed,
        DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
        ModuleConstants.CANivore
    );

    private AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

    SwerveModulePosition[] modulesPosition = {
        FrontLeft.getPosition(),
        FrontRight.getPosition(),
        BackLeft.getPosition(),
        BackRight.getPosition()
    };

    private final SwerveDriveOdometry Odometer = new SwerveDriveOdometry(

        DriveConstants.kDriveKinematics,
        new Rotation2d(0),
        modulesPosition

    );

    private final SwerveDriveOdometry3d odometry3d = new SwerveDriveOdometry3d(
            DriveConstants.kDriveKinematics, 
            new Rotation3d(), 
            modulesPosition
        ); 

    public SwerveSubsystem(){

        this.isFieldOriented = true;

        new Thread(
            ()->{
                try{
                    Thread.sleep(1000);
                    ZeroHeading();

                }catch(Exception e){
                    System.out.println(e);
                    SmartDashboard.putString("Error: ", e.toString());
                }

            }
        ).start();;

    }

    public boolean isFieldOriented(){

        return isFieldOriented;

    }

    public void changeIsFieldOriented( boolean newState ){

        isFieldOriented = newState;

    }

    public void ZeroHeading(){

        gyro.reset();
        resetEncoders();

    }

    public void resetEncoders(){
        FrontLeft.ResetEncoders();
        FrontRight.ResetEncoders();
        BackLeft.ResetEncoders();
        BackRight.ResetEncoders();
    }  

    public void ChangeHeading(double angle){

        gyro.reset();
        gyro.setAngleAdjustment(angle);

    }

    public double getHeading(){

        double heading = Math.IEEEremainder(gyro.getAngle(), 360);

        return heading;

    }

    public Rotation2d getRotation2d(){

        return Rotation2d.fromDegrees(getHeading());

    }

    public Pose2d getPose2d(){

        return Odometer.getPoseMeters();

    }

    public Pose3d getPose3d(){

        return odometry3d.getPoseMeters();

    }

    public void resetOdometry( Pose2d pose ){

        Odometer.resetPosition(getRotation2d(), modulesPosition, pose);

    }

    public void reset3dOdometry (Pose3d pose ){

        odometry3d.resetPosition(gyro.getRotation3d(), modulesPosition, pose);

    }

    @Override
    public void periodic(){

        odometry3d.update(gyro.getRotation3d(), modulesPosition);

        Odometer.update(getRotation2d(), modulesPosition);

        SmartDashboard.putNumber("RobotHeading: 2d Odometry ", getHeading());
        SmartDashboard.putString("Robot Location: 2d Odometry", getPose2d().getTranslation().toString());

        SmartDashboard.putString("Robot Location: 3d Odometry", getPose3d().getTranslation().toString());

    }

    public void stopModules(){

        FrontLeft.Stop();
        FrontRight.Stop();
        BackLeft.Stop();
        BackRight.Stop();

    }

    public void setModuleStates( SwerveModuleState[] desireState ){

        SwerveDriveKinematics.desaturateWheelSpeeds(desireState, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

        FrontLeft.SetDesireState(desireState[0]);
        FrontRight.SetDesireState(desireState[1]);
        BackLeft.SetDesireState(desireState[2]);
        BackRight.SetDesireState(desireState[3]);

    }

}