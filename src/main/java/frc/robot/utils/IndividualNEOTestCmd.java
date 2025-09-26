package frc.robot.utils;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;

public class IndividualNEOTestCmd extends Command {

    private SparkMax testMotor;

    private double velocity;

    public IndividualNEOTestCmd(
        int id,
        MotorType motorType,
        double velocity
    ){
        this.testMotor = new SparkMax(id, motorType);
        this.velocity = velocity;
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){

        testMotor.set(velocity);

    }

    @Override 
    public void end(boolean interrupted){

        testMotor.set(0);

    }

    @Override
    public boolean isFinished(){

        return false;

    }
    
}
