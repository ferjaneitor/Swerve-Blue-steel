package frc.robot.Vision;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.FieldCosntants;

public class superPhotonCamera {

    private final PhotonCamera camera;

    private PhotonPipelineResult pipeLineResults;

    private boolean hasTarget;

    private List<PhotonTrackedTarget> targetList;

    private PhotonTrackedTarget bestTarget;

    private double bestYaw, bestPitch, bestSkew, bestArea, bestPoseAmbiguity;

    private Transform3d bestCameraToTarget, bestAlternateCameraToTarget, cameraToRobot;

    private List<TargetCorner> targetCorners;

    private int bestTargetID;

    private PhotonPoseEstimator poseEstimator;

    public superPhotonCamera (
        String cameraName,
        Transform3d CameraToRobotTransform
    ){

        this.camera = new PhotonCamera(cameraName);

        this.cameraToRobot = CameraToRobotTransform;

        this.poseEstimator = new PhotonPoseEstimator(
            FieldCosntants.kTagLayout
            ,PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
            ,this.cameraToRobot
        );

        this.pipeLineResults = new PhotonPipelineResult();
        this.targetList = Collections.emptyList();
        this.targetCorners = Collections.emptyList();
        this.bestAlternateCameraToTarget = null;
        this.bestCameraToTarget = null ;
        this.bestTargetID = -1;
        this.hasTarget = false;
        this.bestArea = this.bestPitch = this.bestSkew = this.bestYaw = this.bestPoseAmbiguity = Double.NaN;

    }

    public void update() {
        final PhotonPipelineResult latest = camera.getLatestResult();
        this.pipeLineResults = (latest != null) ? latest : new PhotonPipelineResult();

        this.hasTarget = this.pipeLineResults.hasTargets();

        final List<PhotonTrackedTarget> ts = this.pipeLineResults.getTargets();
        this.targetList = (ts != null) ? ts : Collections.emptyList();

        this.bestTarget = this.pipeLineResults.getBestTarget();
        if (this.bestTarget != null) {
            this.bestArea = this.bestTarget.getArea();
            this.bestPitch = this.bestTarget.getPitch();
            this.bestSkew = this.bestTarget.getSkew();
            this.bestYaw = this.bestTarget.getYaw();
            this.bestPoseAmbiguity = this.bestTarget.getPoseAmbiguity();

            this.bestAlternateCameraToTarget = this.bestTarget.getAlternateCameraToTarget();
            this.bestCameraToTarget = this.bestTarget.getBestCameraToTarget();

            final List<TargetCorner> corners = this.bestTarget.getDetectedCorners();
            this.targetCorners = (corners != null) ? corners : Collections.emptyList();

            this.bestTargetID = this.bestTarget.getFiducialId();
        } else {
            clearBest();
        }
    }

    private void clearBest() {
        this.targetCorners = Collections.emptyList();
        this.bestAlternateCameraToTarget = null;
        this.bestCameraToTarget = null;
        this.bestTargetID = -1;
        this.bestArea = this.bestPitch = this.bestSkew = this.bestYaw = this.bestPoseAmbiguity = Double.NaN;
    }

    public PhotonPoseEstimator getPoseEstimator(){

        return this.poseEstimator;

    }
    
    public Pose3d FieldRelativePose ( ){
        if (FieldCosntants.kTagLayout.getTagPose(bestTargetID).isPresent()) {

            return PhotonUtils.estimateFieldToRobotAprilTag(bestCameraToTarget, FieldCosntants.kTagLayout.getTagPose(bestTargetID).get(), this.cameraToRobot);
            
        }else{

            return new Pose3d();

        }
    }

    /** Estima pose con PhotonPoseEstimator (multi-tag). Usa el último frame. */
    public Optional<EstimatedRobotPose> estimateRobotPoseMultiTag() {
        return poseEstimator.update(this.pipeLineResults);
    }

    /**
     * Estimación single-tag con utilitario (sólo si conoces el ID y hay bestCameraToTarget).
     * Devuelve Optional.empty() si no hay datos suficientes.
     */
    public Optional<Pose3d> estimateRobotPoseSingleTag() {
        if (bestCameraToTarget == null || bestTargetID < 0) return Optional.empty();
        final var optTagPose = FieldCosntants.kTagLayout.getTagPose(bestTargetID);
        if (optTagPose.isEmpty()) return Optional.empty();
        Pose3d fieldToRobot = PhotonUtils.estimateFieldToRobotAprilTag(
            bestCameraToTarget, optTagPose.get(), this.cameraToRobot
        );
        return Optional.of(fieldToRobot);
    }

    /**
     * Drena todos los frames no leídos, estima y entrega cada `EstimatedRobotPose`
     * al consumidor (útil para llamar `poseEstimator.addVisionMeasurement`).
     */
    public void consumeUnreadAndEstimate(Consumer<EstimatedRobotPose> consumer) {
        for (var frame : camera.getAllUnreadResults()) {
            poseEstimator.update(frame).ifPresent(consumer);
        }
    }

    public static double getDistanceBetween(Pose2d a, Pose2d b) {
        return PhotonUtils.getDistanceToPose(a, b);
    }

    /** Traducción cámara→objetivo estimada a partir de distancia e Yaw actual. */
    public Optional<Translation2d> estimateCameraToTargetTranslation(double distanceMeters) {
        if (Double.isNaN(bestYaw)) return Optional.empty();
        // Nota: el signo puede depender de tu convención. Ajusta si ves un espejo L/R.
        return Optional.of(PhotonUtils.estimateCameraToTargetTranslation(
            distanceMeters, Rotation2d.fromDegrees(-this.bestYaw)
        ));
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

    public Transform3d getbestAlternateCameraToTarget(){

        return bestAlternateCameraToTarget;

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