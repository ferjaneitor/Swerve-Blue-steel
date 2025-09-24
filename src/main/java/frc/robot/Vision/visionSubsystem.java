package frc.robot.Vision;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class visionSubsystem extends SubsystemBase {

    enum PoseEstimateSource {
        LOW(0.025),
        MEDIUM(0.15),
        HIGH(0.3),
        NONE(99.0);
    
        private Matrix<N3, N1> stdDev;

        PoseEstimateSource(double dev){
            this(dev, dev, dev);
        }
    
        PoseEstimateSource(double xDev, double yDev, double thetaDev) {
            this.stdDev = MatBuilder.fill(Nat.N3(), Nat.N1(), xDev, yDev, thetaDev);
        }
    }    


}
