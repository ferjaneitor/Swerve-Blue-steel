package frc.robot.Drive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
 
    //Declaracion de motores
    private final SparkMax SteeringMotor, DriveMotor;

    //declaracion de los encoders reltivos
    private final RelativeEncoder SteeringMotorEncoder, DriveMotorEncoder;

    //Declaracion de la configuracion de los motores
    private SparkMaxConfig SteeringMotorConfig, DriveMotorConfig;

    //declaracion del encoder absoluto
    private final CANcoder AbsoluteEncoder ;

    //declaracfion del PID
    private final PIDController steeringPidController;

    //declaracion del offset
    private double AbsoluteEncoderOffSet;

    //declaracion para saber si el encoder tiene que estar en reversa
    private boolean AbsoluteEncoderReversed;

    //declaracion para saber el ID del encoder
    private int absoluteEncoderID;

    //funcion para crear a los modulos
    public SwerveModule( 

        //Requisitos
        int SteeringMotorID, 
        int DriveMotorID, 
        int EncoderID, 
        boolean SteeringMotorReversed, 
        boolean DriveMotorReversed, 
        boolean EncoderReversed, 
        double offSet,
        String CANivore

     ){

        //Asignamos todos los valores que queremos conservar a lo largo del codigo
        this.AbsoluteEncoderOffSet = offSet;
        this.AbsoluteEncoderReversed = EncoderReversed;
        this.absoluteEncoderID = EncoderID;

        //Creamos los motores
        DriveMotor = new SparkMax(DriveMotorID, MotorType.kBrushless);
        SteeringMotor = new SparkMax(SteeringMotorID, MotorType.kBrushless);

        //Conseguimos los encoders
        DriveMotorEncoder = DriveMotor.getEncoder();
        SteeringMotorEncoder = SteeringMotor.getEncoder();

        //Creamos el encoder absoluto
        AbsoluteEncoder = new CANcoder(EncoderID, CANivore);

        //creamos las configuraciones 
        DriveMotorConfig = new SparkMaxConfig();
        SteeringMotorConfig = new SparkMaxConfig();

        //hacemos que los motores esten en brake
        DriveMotorConfig.idleMode(IdleMode.kBrake);
        SteeringMotorConfig.idleMode(IdleMode.kBrake);
        
        //invertimos si es necesario los motores
        DriveMotorConfig.inverted(DriveMotorReversed);
        SteeringMotorConfig.inverted(SteeringMotorReversed);

        //Sacamos la convesion de posicion para los encoders
        DriveMotorConfig.absoluteEncoder.positionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        SteeringMotorConfig.absoluteEncoder.positionConversionFactor(ModuleConstants.kSteeringEncoderRot2Rad);

        //sacamos la conversion de velocidad para los motores
        DriveMotorConfig.absoluteEncoder.velocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        SteeringMotorConfig.absoluteEncoder.velocityConversionFactor(ModuleConstants.kSteeringEncoderRPM2RadPerSec);

        //aplciamos la configuracion a los motores
        DriveMotor.configure(DriveMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        SteeringMotor.configure(SteeringMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        //creamos el PID
        steeringPidController = new PIDController(ModuleConstants.kPSteering, ModuleConstants.kISteering, ModuleConstants.KDSteering);

        //Activamos que el PID pueda continuar con un flujo constante
        steeringPidController.enableContinuousInput(-Math.PI, Math.PI);

    }

    public double getDrivePosition(){

        return DriveMotorEncoder.getPosition();

    }

    public double getSteeringPosition(){

        return SteeringMotorEncoder.getPosition();

    }

    public double getDriveVelocity(){

        return DriveMotorEncoder.getVelocity();

    }

    public double getSteeringVelocity(){

        return SteeringMotorEncoder.getVelocity();

    }

    public double getAbsolutePosition(){

        double angle = AbsoluteEncoder.getAbsolutePosition().getValueAsDouble();
        
        double radOffSet = AbsoluteEncoderOffSet;

        angle *= 2;
        angle *= Math.PI;

        radOffSet *= 2;
        radOffSet *= Math.PI;

        angle -= radOffSet;

        return angle * (AbsoluteEncoderReversed ? -1.0 : 1.0);

    }

    public void ResetEncoders(){

        double encoderPosition = getAbsolutePosition();

        DriveMotorEncoder.setPosition(0);
        SteeringMotorEncoder.setPosition(encoderPosition);

        SmartDashboard.putNumber("Encoder: "+ absoluteEncoderID +" ", encoderPosition);

    }

    public SwerveModuleState getState(){

        SwerveModuleState State = new SwerveModuleState(getDriveVelocity(), new Rotation2d(getSteeringPosition()));

        return State;

    }

    public SwerveModulePosition getPosition(){

        SwerveModulePosition Position = new SwerveModulePosition(getDrivePosition(), new Rotation2d(getSteeringPosition()));

        return Position;

    }

    public void Stop(){

        DriveMotor.setVoltage(0);
        SteeringMotor.setVoltage(0);

    }

    public void SetDesireState( SwerveModuleState state ){

        double StateVelocity = state.speedMetersPerSecond;

        double PhysicalMaxSpeed = DriveConstants.kPhysicalMaxSpeedMetersPerSecond;

        double StateAngleRad = state.angle.getRadians();

        if ( Math.abs( StateVelocity ) < 0.001 ){

            Stop();
            return;

        }

        state.optimize(getState().angle);

        double DriveMotorState = StateVelocity / PhysicalMaxSpeed;

        double SteeringMotorPosition = steeringPidController.calculate(getSteeringPosition(), StateAngleRad );

        DriveMotor.setVoltage( DriveMotorState );
        SteeringMotor.setVoltage(SteeringMotorPosition);

    }

}
