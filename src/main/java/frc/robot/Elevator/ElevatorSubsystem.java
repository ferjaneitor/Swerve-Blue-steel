package frc.robot.Elevator;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ElevatorSubsystem {

    private SparkMax elevatorSpark, pivotspark;

    public ElevatorSubsystem ( int elevatorId, int pivotId) {

        this.elevatorSpark = new SparkMax(elevatorId, MotorType.kBrushed);

        this.pivotspark = new SparkMax(pivotId, MotorType.kBrushed);

    }

    public void setElevator( double speed ){

        this.elevatorSpark.set(speed);

    }

    public void stopElevator () {

        this.elevatorSpark.set(0);

    }

    public void setPivot ( double speed ){

        this.pivotspark.set(speed);

    }

    public void stopPivor(){

        this.pivotspark.set(0);

    }

}