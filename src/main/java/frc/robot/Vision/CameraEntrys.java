package frc.robot.Vision;

import java.util.Objects;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Vision.visionSubsystem.PoseEstimateSource;

public class CameraEntrys {
    
    public static final class CameraEntry{

        public final superPhotonCamera camera;

        public final PoseEstimateSource StdDevs;

        public CameraEntry( superPhotonCamera camera, PoseEstimateSource Std ){

            this.StdDevs = Objects.requireNonNull(Std);

            this.camera = Objects.requireNonNull(camera);

        }

    }

    public static final class CameraSpecs {

        public final String CameraName;

        public final Transform3d CameraToRobot;

        public final PoseEstimateSource StdDevs;

        public CameraSpecs(
            String CameraName,
            Transform3d CameraToRobot,
            PoseEstimateSource StdDevs
        ){

            this.CameraName = CameraName;
            this.CameraToRobot = CameraToRobot;
            this.StdDevs = StdDevs;
            

        }

    }

}
