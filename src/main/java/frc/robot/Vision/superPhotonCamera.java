package frc.robot.Vision;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;

public class superPhotonCamera {

    private final PhotonCamera camera;

    private PhotonPipelineResult pipeLineResults;

    private boolean hasTarget;

    private List<PhotonTrackedTarget> targetList;

    private PhotonTrackedTarget bestTarget;

    private double bestYaw, bestPitch, bestSkew, bestArea, bestPoseAmbiguity;

    private Transform3d bestCameraToTarget, bestAltarnateCameraToTarget;

    private List<TargetCorner> targetCorners;

    private int bestTargetID;

    public superPhotonCamera (
        String cameraName
    ){

        this.camera = new PhotonCamera(cameraName);

        this.pipeLineResults = new PhotonPipelineResult();
        this.targetList = List.of();
        this.targetCorners = List.of();
        this.bestAltarnateCameraToTarget = null;
        this.bestCameraToTarget = null ;
        this.bestTargetID = -1;
        this.hasTarget = false;
        this.bestArea = this.bestPitch = this.bestSkew = this.bestYaw = this.bestPoseAmbiguity = Double.NaN;

    }

    public void update(){

        PhotonPipelineResult pipeLineUpdate = camera.getLatestResult();

        if( pipeLineUpdate == null ){

            this.pipeLineResults = new PhotonPipelineResult();

        }else{

            this.pipeLineResults = pipeLineUpdate;

        }

        this.hasTarget = pipeLineResults.hasTargets();

        List<PhotonTrackedTarget> ts = pipeLineResults.getTargets();
        this.targetList = ( ts != null ) ? ts : List.of();

        this.bestTarget = pipeLineResults.getBestTarget();

        if ( this.bestTarget != null) {
            
            this.bestArea = this.bestTarget.getArea();
            this.bestPitch = this.bestTarget.getPitch();
            this.bestSkew = this.bestTarget.getSkew();
            this.bestYaw = this.bestTarget.getYaw();
            this.bestPoseAmbiguity = this.bestTarget.getPoseAmbiguity();

            this.bestAltarnateCameraToTarget = this.bestTarget.getAlternateCameraToTarget();
            this.bestCameraToTarget = this.bestTarget.getBestCameraToTarget();

            List<TargetCorner> corners = this.bestTarget.getDetectedCorners();
            this.targetCorners = ( corners !=null ) ? corners : List.of() ;

            this.bestTargetID = this.bestTarget.getFiducialId();

        } else {
            
            this.targetCorners = List.of();
            this.bestAltarnateCameraToTarget = null;
            this.bestCameraToTarget = null ;
            this.bestTargetID = -1;
            this.bestArea = this.bestPitch = this.bestSkew = this.bestYaw = this.bestPoseAmbiguity = Double.NaN;

        }

    }

    public boolean isConnected(){

        return camera.isConnected();

    }

    public boolean getDriverMode(){

        return camera.getDriverMode();

    }

    public void setDriverMode( boolean state ){

        camera.setDriverMode(state);

    }

    public double getLatencyMillis() {
        if (this.pipeLineResults == null) return Double.NaN;
        // Si tu versión de PhotonVision tiene getLatencyMillis(), úsalo:
        // return pipelineResult.getLatencyMillis();
        // Alternativa robusta: tiempo actual - timestamp de captura
        double seconds = Timer.getFPGATimestamp() - this.pipeLineResults.getTimestampSeconds();
        return seconds * 1000.0;
    }

    public double getLatencySeconds(){

        return ( pipeLineResults != null ) ? pipeLineResults.getTimestampSeconds() : Double.NaN; 

    }

    public PhotonPipelineResult getPhotonPipelineResult (){

        return pipeLineResults;

    }

    public boolean getHasTarget(){

        return hasTarget;

    }

    public List<PhotonTrackedTarget> gettarget(){

        return targetList;

    }

    public PhotonTrackedTarget getBesTarget(){

        return bestTarget;

    }

    public double getBestYaw(){

        return bestYaw;

    }

    public double getBestPitch(){

        return bestPitch;

    }

    public double getbestSkew(){

        return bestSkew;

    }

    public double getBestArea(){

        return bestArea;

    }

    public double getBestPoseAmbiguity(){

        return bestPoseAmbiguity;

    }

    public Transform3d getBestCameraTotarget(){

        return bestCameraToTarget;

    }

    public Transform3d getBestAltarnateCameraToTarget(){

        return bestAltarnateCameraToTarget;

    }

    public List<TargetCorner> getTargetCorners(){

        return targetCorners;

    }

    public int getBestTargetID(){

        return bestTargetID;

    }

    public List<Double> getYawList(){

        List<Double> outPut = new ArrayList<>(targetList.size());

        for ( PhotonTrackedTarget target : targetList ) outPut.add(target.getYaw());

        return outPut;

    }

    public List<Double> getPitchList(){

        List<Double> outPut = new ArrayList<>(targetList.size());

        for ( PhotonTrackedTarget target : targetList ) outPut.add(target.getPitch());

        return outPut;

    }

    public List<Double> getAreaList(){

        List<Double> outPut = new ArrayList<>(targetList.size());

        for ( PhotonTrackedTarget target : targetList ) outPut.add(target.getArea());

        return outPut;

    }

    public List<Double> getSkewList(){

        List<Double> outPut = new ArrayList<>(targetList.size());

        for ( PhotonTrackedTarget target : targetList ) outPut.add(target.getSkew());

        return outPut;

    }

    public List<Double> getPoseAmbiguityList(){

        List<Double> outPut = new ArrayList<>(targetList.size());

        for ( PhotonTrackedTarget target : targetList ) outPut.add(target.getPoseAmbiguity());

        return outPut;

    }

    public List<Transform3d> getBestCameraToTargetList(){

        List<Transform3d> outPut = new ArrayList<>(targetList.size());

        for ( PhotonTrackedTarget target : targetList ) outPut.add(target.getBestCameraToTarget());

        return outPut;

    }

    public List<Transform3d> getAltarnateCameraToTargetList(){

       List<Transform3d> outPut = new ArrayList<>(targetList.size());

        for ( PhotonTrackedTarget target : targetList ) outPut.add(target.getAlternateCameraToTarget());

        return outPut;

    }

    public List<List<TargetCorner>> getTargetCornersList(){

        List<List<TargetCorner>> outPut = new ArrayList<>(targetList.size());

        for ( PhotonTrackedTarget target : targetList ) outPut.add(target.getDetectedCorners());

        return outPut;

    }

    public List<Integer> getTargetIDList(){

        List<Integer> outPut = new ArrayList<>(targetList.size());

        for ( PhotonTrackedTarget target : targetList ) outPut.add(target.getFiducialId());

        return outPut;

    }
    
}