package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.util.MathUtil;

import java.util.List;

/**
 * Webcam AprilTag vision for the Logitech camera configured as "logitech".
 *
 * This keeps the same simple data shape the old Limelight code used:
 * hasTarget, tx, ty, and distance.
 */
public class LogitechCameraSubsystem {

    private final WebcamName camera;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private AprilTagDetection latest = null;
    private long lastGoodMs = 0;

    public long staleMs = 250;
    public double distanceFilterAlpha = 0.85;

    private double filteredDistanceIn = 70.0;

    public LogitechCameraSubsystem(WebcamName camera) {
        this.camera = camera;
    }

    /** Call once in OpMode.init() */
    public void init() {
        aprilTag = new AprilTagProcessor.Builder().build();

        // Lower decimation sees farther, higher decimation runs faster.
        aprilTag.setDecimation(2);

        visionPortal = new VisionPortal.Builder()
                .setCamera(camera)
                .addProcessor(aprilTag)
                .build();
    }

    /** Call every loop */
    public void update() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null && detection.ftcPose != null) {
                latest = detection;
                lastGoodMs = System.currentTimeMillis();

                double range = detection.ftcPose.range;
                if (!Double.isNaN(range) && !Double.isInfinite(range) && range > 0.1 && range < 250.0) {
                    filteredDistanceIn = MathUtil.lowPass(filteredDistanceIn, range, distanceFilterAlpha);
                }
                return;
            }
        }
    }

    public boolean hasTarget() {
        if (latest == null) return false;
        return (System.currentTimeMillis() - lastGoodMs) <= staleMs;
    }

    public int getPipelineIndex() {
        return hasTarget() ? 0 : -1;
    }

    public double getTxDeg() {
        return hasTarget() ? latest.ftcPose.bearing : 0.0;
    }

    public double getTyDeg() {
        return hasTarget() ? latest.ftcPose.elevation : 0.0;
    }

    public double getDistanceInches() {
        return filteredDistanceIn;
    }

    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
            visionPortal = null;
        }
    }
}
