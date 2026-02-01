package frc.robot.util.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

import org.photonvision.PhotonCamera; 
import edu.wpi.first.math.util.Units;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.Constants;
import frc.robot.util.EstimatedRobotPose;
import frc.robot.util.FieldUtil;
import frc.robot.util.vision.PhotonUtil.BW;
import frc.robot.util.vision.PhotonUtil.BW.BWCamera;

import java.util.List;
import java.util.Optional;
//import java.util.concurrent.atomic.AtomicReference;


public final class PhotonUtil {
    public static void updateResults(Rotation2d heading){ 
        BW.updateResults(heading);

    }

    public static final class Color{
        public enum TargetClass {
            FUEL(0),
            NONE(-1);

            public final int id;

            TargetClass(int id) {
                this.id = id;
            }
            public static Color.TargetClass fromID(int id) {
                return switch (id) {
                    case 0 -> FUEL;
                    default -> NONE;
                };
            }
        }

        private static final PhotonCamera COLOR = new PhotonCamera(Constants.NT_INSTANCE, Constants.VisionConstants.PhotonConstants.COLOR_NAME);
        private static PhotonPipelineResult latestResult = new PhotonPipelineResult();

        public static boolean hasTargets() {
            return latestResult.hasTargets();
        }

        private static final Transform3d robotToCameraOffset = new Transform3d(
                Constants.VisionConstants.PhotonConstants.COLOR_FORWARD,
                Constants.VisionConstants.PhotonConstants.COLOR_LEFT,
                Constants.VisionConstants.PhotonConstants.COLOR_UP,
                new Rotation3d(
                        Units.degreesToRadians(Constants.VisionConstants.PhotonConstants.COLOR_ROLL),
                        Units.degreesToRadians(Constants.VisionConstants.PhotonConstants.COLOR_PITCH),
                        Units.degreesToRadians(Constants.VisionConstants.PhotonConstants.COLOR_YAW)
                )
            );
                private static final List<PhotonTrackedTarget> results = null;
                public static boolean hasTargets(Color.TargetClass targetClass) {
                    if (hasTargets()) {
                        for (PhotonTrackedTarget target : latestResult.getTargets()) {
                            if (target.getDetectedObjectClassID() == targetClass.id) {
                                return true;
                            }
                        }
                    }
                    return false;
                }
        
                public static Optional<PhotonTrackedTarget> getBestObject() {
                    return hasTargets() ? Optional.of(latestResult.getBestTarget()) : Optional.empty();
                }
                
                public static Optional<PhotonTrackedTarget> getBestObject(Color.TargetClass targetClass) {
                    if(hasTargets(targetClass)){ 
                        for (PhotonTrackedTarget target : latestResult.getTargets()) {
                            if (target.getDetectedObjectClassID() == targetClass.id) {
                                return Optional.of(target);
                            }
                        }
                    }
                    return Optional.empty();
                }
        
                public static Color.TargetClass getBestObjectClass() {
                    return getBestObject().map(bestObject -> TargetClass.fromID(bestObject.getDetectedObjectClassID()))
                    .orElse(Color.TargetClass.NONE);
                }
        
                public static double getBestObjectYaw() {
                    return getBestObject().map(PhotonTrackedTarget::getYaw).orElse(0.0);
                }
        
                public static double getBestObjectPitch() {
                    return getBestObject().map(PhotonTrackedTarget::getPitch).orElse(0.0);
                }
        
                public static double getBestObjectYaw(Color.TargetClass targetClass) {
                    return getBestObject(targetClass).map(PhotonTrackedTarget::getYaw).orElse(0.0);
                }
        
                public static double getBestObjectPitch(Color.TargetClass targetClass) {
                    return getBestObject(targetClass).map(PhotonTrackedTarget::getPitch).orElse(0.0);
                }
        
                public static Optional<Translation2d> getRobotToBestObject(TargetClass targetClass) {
                if (!hasTargets(targetClass)|| getBestObject(targetClass).isEmpty()) {
        
                }
                PhotonTrackedTarget bestTarget = getBestObject(targetClass).get();
        
                Translation2d camToObjectTranslation = new Pose3d(
                    Translation3d.kZero,
                    new Rotation3d(
                        0,
                        -Math.toRadians(bestTarget.getPitch()),
                        -Math.toRadians(bestTarget.getYaw())
                    )
                ).transformBy(
                            new Transform3d(
                                new Translation3d(bestTarget.getBestCameraToTarget().getTranslation().getNorm(), 0, 0),
                                Rotation3d.kZero
                            )        
                ).getTranslation().rotateBy(
                    new Rotation3d(
                        robotToCameraOffset.getRotation().getX(),
                        robotToCameraOffset.getRotation().getY(),
                        0
                    )
                ).toTranslation2d();
        
                return Optional.of(new Translation2d(robotToCameraOffset.getX(), robotToCameraOffset.getY())
                        .plus(camToObjectTranslation.rotateBy(robotToCameraOffset.getRotation().toRotation2d().unaryMinus())));
                }
        
                public static void updateResults(){
                    List<PhotonPipelineResult> result = COLOR.getAllUnreadResults();
                    if(!results.isEmpty()) {
                        latestResult = result.get(results.size() -1);
             }
        }
    }

    public static final class BW { 
        private static final TimeInterpolatableBuffer<Rotation2d> headingBuffer = 
                TimeInterpolatableBuffer.createBuffer(1.0);

        public enum BWCamera {
    
            FRONT(
                    new PhotonCamera(Constants.NT_INSTANCE, Constants.VisionConstants.PhotonConstants.BW_FRONT_NAME),
                    new Transform3d(
                        Constants.VisionConstants.PhotonConstants.BW_FRONT_FORWARD,
                        Constants.VisionConstants.PhotonConstants.BW_FRONT_LEFT,
                        Constants.VisionConstants.PhotonConstants.BW_FRONT_UP,
                            new Rotation3d(
                                Units.degreesToRadians(Constants.VisionConstants.PhotonConstants.BW_FRONT_ROLL),
                                Units.degreesToRadians(Constants.VisionConstants.PhotonConstants.BW_FRONT_PITCH),
                                Units.degreesToRadians(Constants.VisionConstants.PhotonConstants.BW_FRONT_YAW)

                            )
                    )
            ),
            BACK(
                    new PhotonCamera(Constants.NT_INSTANCE, Constants.VisionConstants.PhotonConstants.BW_BACK_NAME),
                    new Transform3d(
                        Constants.VisionConstants.PhotonConstants.BW_BACK_FORWARD,
                        Constants.VisionConstants.PhotonConstants.BW_BACK_LEFT,
                        Constants.VisionConstants.PhotonConstants.BW_BACK_UP,
                            new Rotation3d(
                                Units.degreesToRadians(Constants.VisionConstants.PhotonConstants.BW_BACK_ROLL),
                                Units.degreesToRadians(Constants.VisionConstants.PhotonConstants.BW_BACK_PITCH),
                                Units.degreesToRadians(Constants.VisionConstants.PhotonConstants.BW_BACK_YAW)

                            )
                    )
            );

            final PhotonCamera camera; 
            final Transform3d robotToCameraOffset;
            
            BWCamera(PhotonCamera camera, Transform3d robotToCameraOffset) {
                this.camera = camera;
                this.robotToCameraOffset = robotToCameraOffset;

            }

            public PhotonCamera getCamera() {
                return camera;
            }

            public Transform3d getRobotToCameraOffset() {
                return robotToCameraOffset;
            }
        }

        private static PhotonPipelineResult latestResultFront = new PhotonPipelineResult();
        private static PhotonPipelineResult latestResultBack = new PhotonPipelineResult();

        public static PhotonPipelineResult getLatestResult(BWCamera camera) {
            return switch (camera) {
                case FRONT -> latestResultFront; 
                case BACK -> latestResultBack;
            };
        }

        public static boolean hasTargets(BWCamera camera) {
            return switch (camera) {
                case FRONT -> latestResultFront.hasTargets();
                case BACK -> latestResultBack.hasTargets();
            };
        }

        public static double getLatestResultTimestamp(BWCamera camera) {
            return switch (camera) {
                case FRONT -> latestResultFront.getTimestampSeconds(); 
                case BACK -> latestResultBack.getTimestampSeconds();
            };
        }

        public static double getBestTagID(BWCamera camera) {
            return hasTargets(camera) ? getLatestResult(camera).getBestTarget().getFiducialId() : 0.0;
        }
        
        public static double getBestTagDist(BWCamera camera) {
            return hasTargets(camera)
                    ? getLatestResult(camera).getBestTarget().getBestCameraToTarget().getTranslation().toTranslation2d().getNorm()
                    : 0.0;
        }

        public static boolean isMultiTag(BWCamera camera) {
            return getLatestResult(camera).getTargets().size() >=2;
        }

        public static boolean isTagClear(BWCamera camera) {
            return hasTargets(camera) && getBestTagDist(camera) < Constants.VisionConstants.PhotonConstants.BW_MAX_TAG_CLEAR_DIST;
        }

        public static boolean isTagClear() {
            return isTagClear(BWCamera.FRONT) || isTagClear(BWCamera.BACK);
        }

        public static Optional<EstimatedRobotPose> getMultiTagPose(BWCamera camera) {
            Optional<MultiTargetPNPResult> multiTagResult = getLatestResult(camera).getMultiTagResult();
            return multiTagResult.map(
                    multiTargetPNPResult -> {
                        Pose3d bestPose = new Pose3d().plus(multiTargetPNPResult.estimatedPose.best).relativeTo(FieldUtil.ORIGIN).plus(camera.robotToCameraOffset.inverse());
                        return new EstimatedRobotPose(
                            bestPose,
                            getLatestResultTimestamp(camera),
                            getLatestResult(camera).getTargets(),
                            Constants.VisionConstants.VISION_STD_DEV_MULTITAG_FUNCTION.apply(getBestTagDist(camera))
                        );
                    }
            );
        }

        public static Optional<EstimatedRobotPose> getSingleTagPose(BWCamera camera, Pose2d currentPose){
            if (!hasTargets(camera)) {
                return Optional.empty();
            }

            Pose3d tagPose = FieldUtil.AprilTag.getTag(getBestTagID(camera)).pose3d;
            PhotonPipelineResult result = getLatestResult(camera);

            Transform3d cameraToTargetBest = result.getBestTarget().getBestCameraToTarget();
            Transform3d cameraToTargetAlt = result.getBestTarget().getAlternateCameraToTarget();

            double bestDist = cameraToTargetBest.getTranslation().toTranslation2d().getNorm();
            double altDist = cameraToTargetAlt.getTranslation().toTranslation2d().getNorm();

            Pose3d poseBest = tagPose.plus(cameraToTargetBest.inverse()).relativeTo(FieldUtil.ORIGIN).plus(camera.robotToCameraOffset.inverse());
            Pose3d poseAlt = tagPose.plus(cameraToTargetAlt.inverse()).relativeTo(FieldUtil.ORIGIN).plus(camera.robotToCameraOffset.inverse());

            Pose3d poseToReturn;
            double distToApply;

            //tweak numbers :(
            //pose determiation
            double ambiguity = getLatestResult(camera).getBestTarget().getPoseAmbiguity();
            if (ambiguity < 0.15) {
                poseToReturn = poseBest; 
                distToApply = bestDist;
            } else if (Math.abs(poseBest.toPose2d().getRotation().minus(currentPose.getRotation()).getDegrees())
                < Math.abs(poseAlt.toPose2d().getRotation().minus(currentPose.getRotation()).getDegrees())  
                && ambiguity < 0.4) {
            poseToReturn = poseBest;
            distToApply = bestDist;
            }else if (ambiguity < 0.4) {
                poseToReturn = poseBest;
                distToApply = bestDist; 
            }else if (ambiguity < 0.4) {
                poseToReturn = poseAlt;
                distToApply = altDist;
            }else {
                return Optional.empty();
            }

            return Optional.of(new EstimatedRobotPose(
                    poseToReturn,
                    result.getTimestampSeconds(),
                    result.getTargets(),
                    Constants.VisionConstants.VISION_STD_DEV_FUNCTION.apply(getBestTagDist(camera))
            ));
        }

        private static Optional<EstimatedRobotPose> getSingleTagPose(BWCamera camera) {
            if (!hasTargets(camera)) {
                return Optional.empty();
            }
            PhotonPipelineResult result = getLatestResult(camera);
            PhotonTrackedTarget bestTarget = result.getBestTarget();

            Translation2d camToTagTranslation = new Pose3d(
                    Translation3d.kZero,
                    new Rotation3d(
                            0,
                            -Math.toRadians(bestTarget.getPitch()),
                            -Math.toRadians(bestTarget.getYaw())

                    )
            ).transformBy(
                    new Transform3d(
                            new Translation3d(bestTarget.getBestCameraToTarget().getTranslation().getNorm(), 0, 0),
                            Rotation3d.kZero
                    )
            ).getTranslation().rotateBy(
                    new Rotation3d(
                            camera.robotToCameraOffset.getRotation().getX(),
                            camera.robotToCameraOffset.getRotation().getY(),
                            0
                    )
            ).toTranslation2d();

            if(headingBuffer.getSample(result.getTimestampSeconds()).isEmpty()) {
                return Optional.empty();
            }

            Rotation2d headingSample = headingBuffer.getSample(result.getTimestampSeconds()).get();

            Rotation2d camToTagRotation = headingSample.plus(
                    camera.robotToCameraOffset.getRotation().toRotation2d().plus(camToTagTranslation.getAngle())
            );

            if (FieldUtil.layout2025.getTagPose(bestTarget.getFiducialId()).isEmpty()) {
                return Optional.empty();
            }
                
            FieldUtil.AprilTag tag = FieldUtil.AprilTag.getTag(bestTarget.getFiducialId());

            if (!FieldUtil.AprilTag.FILTER.contains(tag)) {
                return Optional.empty();
            }

            Pose2d tagPose2d = tag.pose2d;

            Translation2d fieldToCameraTranslation = 
                    new Pose2d(tagPose2d.getTranslation(), camToTagRotation.plus(Rotation2d.kPi))
                            .transformBy(new  Transform2d(camToTagTranslation.getNorm(), 0, Rotation2d.kZero))
                            .getTranslation();
                             Pose2d robotPose = new Pose2d(
                    new Pose2d(
                            fieldToCameraTranslation,
                            headingSample.plus(camera.robotToCameraOffset.getRotation().toRotation2d())
                    ).transformBy(
                            new Transform2d(
                                    new Pose3d(
                                            camera.robotToCameraOffset.getTranslation(),
                                            camera.robotToCameraOffset.getRotation()
                                    ).toPose2d(),
                                    Pose2d.kZero
                            )
                    ).getTranslation(),
                    headingSample
            );

            return Optional.of(
                    new EstimatedRobotPose(
                            new Pose3d(robotPose),
                            result.getTimestampSeconds(),
                            result.getTargets(),
                            Constants.VisionConstants.VISION_STD_DEV_FUNCTION.apply(camToTagTranslation.getNorm())
                    )
            );
        }
   
        public static Optional<EstimatedRobotPose> getBestTagPose(BWCamera camera) {
            return BW.isMultiTag(camera) ? getMultiTagPose(camera) : getSingleTagPose(camera);
        }

        public static void updateResults(Rotation2d heading) {
            headingBuffer.addSample(Timer.getFPGATimestamp(), heading);
            for (BWCamera camera : BWCamera.values()) {
                List<PhotonPipelineResult> results = camera.camera.getAllUnreadResults();

                if (!results.isEmpty()) {
                    switch (camera) {
                        case FRONT -> latestResultFront = results.get(results.size() - 1);
                        case BACK -> latestResultBack = results.get(results.size() - 1);
                    
                    }
                }
            }
        }
    }
}