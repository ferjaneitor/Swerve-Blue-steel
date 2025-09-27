package frc.robot.Vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.FieldCosntants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Drive.SwerveSubsystem;
import frc.robot.Vision.CameraEntrys.CameraEntry;
import frc.robot.Vision.CameraEntrys.CameraSpecs;

public class visionSubsystem extends SubsystemBase {

    public enum PoseEstimateSource {
        LOW(0.025),
        MEDIUM(0.15),
        HIGH(0.3),
        NONE(99.0);
    
        @SuppressWarnings("unused")
        private Matrix<N3, N1> stdDev;

        PoseEstimateSource(double dev){
            this(dev, dev, dev);
        }
    
        PoseEstimateSource(double xDev, double yDev, double thetaDev) {
            this.stdDev = MatBuilder.fill(Nat.N3(), Nat.N1(), xDev, yDev, thetaDev);
        }
    }    

    List<CameraEntry> cameras = new ArrayList<CameraEntry>();

    private boolean AllowVision = true;

    private Supplier<Double> yawRateSiSupplier;

    private SwerveSubsystem swerveSubsystem;

    public visionSubsystem (
        Supplier<Double> yawRateSupplier,
        List<CameraSpecs> cameraSpecsList,
        SwerveSubsystem swerveSubsystem
    ){

        this.yawRateSiSupplier = yawRateSupplier; 
        this.swerveSubsystem = swerveSubsystem;
        
        for ( CameraSpecs Source : cameraSpecsList ) addCamera(Source);

    }


    public void addCamera( CameraSpecs cameraSpecs){

        superPhotonCamera camera = new superPhotonCamera(cameraSpecs.CameraName, cameraSpecs.CameraToRobot); 
        PoseEstimateSource StdDevs = cameraSpecs.StdDevs;

        cameras.add(new CameraEntry(camera, StdDevs));

    }

    public void addCameras ( List<CameraSpecs> cameraSpecsList){

        for ( CameraSpecs source : cameraSpecsList ) addCamera(source);

    }

    @Override
    public void periodic(){

        if ( cameras.isEmpty() || !AllowVision ) return;

        for ( CameraEntry IndividualCamera : cameras ){

            IndividualCamera.camera.update();

            IndividualCamera.camera.consumeUnreadAndEstimate(
                EstimatedRobotPose -> {

                    if (!isPoseEstimateValid(EstimatedRobotPose)) return;

                    Matrix<N3,N1> std = tuneStdDev(IndividualCamera.StdDevs.stdDev, EstimatedRobotPose);

                    Pose2d measurment = EstimatedRobotPose.estimatedPose.toPose2d();

                    swerveSubsystem.AddVisionMeasurment( measurment, EstimatedRobotPose.timestampSeconds, std);

                }
            );

        }

    }

    private boolean isPoseEstimateValid ( EstimatedRobotPose estimatedRobotPose){

        if (estimatedRobotPose == null || estimatedRobotPose.estimatedPose == null) return false;

        int targetsUsed = ( estimatedRobotPose != null ) ? estimatedRobotPose.targetsUsed.size() : 0;
        if ( targetsUsed <= 0 ) return false;

        for ( PhotonTrackedTarget targets : estimatedRobotPose.targetsUsed ){

            double ambiguity = targets.getPoseAmbiguity();

            if ( !Double.isNaN(ambiguity) && ambiguity > VisionConstants.maxAmbiguity) return false;

        }

        double avarageDistance = AvarageDistanceMeters(estimatedRobotPose);

        if( Double.isInfinite(avarageDistance)) return false;

        if( targetsUsed == 1 && avarageDistance > VisionConstants.MAX_DIST_SINGLE ) return false;

        if( targetsUsed >= 2 && avarageDistance > VisionConstants.MAX_DIST_MULTIPLE ) return false;

        double yawRate = Math.abs( yawRateSiSupplier.get() );

        if( VisionConstants.MAX_YAW_RATE_DEG_SEC > 0 && yawRate > VisionConstants.MAX_YAW_RATE_DEG_SEC ) return false;

        return true;

    }

    private double AvarageDistanceMeters ( EstimatedRobotPose estimatedRobotPose ){

        if (estimatedRobotPose == null || estimatedRobotPose.estimatedPose == null) return Double.POSITIVE_INFINITY;

        double sum = 0.0;

        int count = 0;

        for ( PhotonTrackedTarget targets: estimatedRobotPose.targetsUsed ){

            Optional<Pose3d> tag = FieldCosntants.kTagLayout.getTagPose(targets.getFiducialId());

            if ( tag.isPresent() ) {

                double distance = tag.get().toPose2d().getTranslation().getDistance(
                                        estimatedRobotPose.estimatedPose.toPose2d().getTranslation()
                                    );
                
                sum += distance;

                count ++;

            }

        }

        return ( count > 0) ? sum / count : Double.POSITIVE_INFINITY;

    }

    private Matrix<N3,N1> tuneStdDev ( Matrix<N3, N1> Base, EstimatedRobotPose estimatedRobotPose){

        int n = ( estimatedRobotPose.targetsUsed != null ) ? estimatedRobotPose.targetsUsed.size() : 1;

        double distance = AvarageDistanceMeters(estimatedRobotPose);

        double scale = 1.0 ;

        if ( n>= 2 ) scale *= 0.7;
        
        if ( distance > 4.0 ) {

            double times = Math.min(1.0, (distance - 4) / 2.0 );
            scale *= (1.0 + times);

        }

        double sx = Base.get(0, 0) * scale;
        double sy = Base.get(1, 0) * scale;
        double st = Base.get(2, 0) * scale;

        return MatBuilder.fill(Nat.N3(), Nat.N1(), sx, sy, st);

    }

    public Command blockVisionUpdates(){

        return Commands.runEnd(
            ()-> AllowVision = false, 
            ()-> AllowVision = true, 
            this
        );

    }

    public void setAllowVision (boolean State){

        AllowVision = State;

    }

}