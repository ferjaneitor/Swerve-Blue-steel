package frc.robot.Drive;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;

public class SwerveJoystickCmd extends Command {
    
    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSupplier,ySupplier,zSupplier;
    private final Supplier<Boolean> fieldorientedSupplier;
    private final SlewRateLimiter xLimiter, yLimiter, zLimiter;

    public SwerveJoystickCmd(
        SwerveSubsystem swerveSubsystem,
        Supplier<Double> xSupplierJoyStick,
        Supplier<Double> ySupplierJoyStick,
        Supplier<Double> zSupplierJoyStick,
        Supplier<Boolean> fieldorientedButton
    ){

        this.swerveSubsystem = swerveSubsystem;
        this.xSupplier = xSupplierJoyStick;
        this.ySupplier = ySupplierJoyStick;
        this.zSupplier = zSupplierJoyStick;
        this.fieldorientedSupplier = fieldorientedButton;

        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.zLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

        addRequirements(swerveSubsystem);

    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){

        double xSpeed = xSupplier.get();
        double ySpeed = -ySupplier.get();
        double zSpeed = zSupplier.get();

        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0;
        zSpeed = Math.abs(zSpeed) > OIConstants.kDeadband ? zSpeed : 0;

        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        zSpeed = zLimiter.calculate(zSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        ChassisSpeeds chassisSpeeds;

        if ( fieldorientedSupplier.get() ){

            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, zSpeed);

            swerveSubsystem.changeIsFieldOriented(false);

        }else{

            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, zSpeed);

            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, swerveSubsystem.getRotation2d());

            swerveSubsystem.changeIsFieldOriented(true);

        }

        SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        swerveSubsystem.setModuleStates(swerveModuleStates);

    }

    @Override 
    public void end(boolean interrupted){

        swerveSubsystem.stopModules();

    }

    @Override
    public boolean isFinished(){

        return false;

    }

}
