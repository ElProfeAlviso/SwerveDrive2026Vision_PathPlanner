package com.team5959.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVisionSubsystem extends SubsystemBase {

    private final PhotonCamera camera;

    public PhotonVisionSubsystem() {
        // Debe coincidir con el nombre configurado en PhotonVision
        camera = new PhotonCamera("LeftAprilTagCamera");
    }

    @Override
    public void periodic() {

        PhotonPipelineResult result = camera.getLatestResult();

        // ¿Hay target?
        SmartDashboard.putBoolean("PV/Has Target", result.hasTargets());

        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();

            // Get information from target.
            int targetID = target.getFiducialId();
            double poseAmbiguity = target.getPoseAmbiguity();
            Transform3d bestCameraToTarget = target.getBestCameraToTarget();
            Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();

            SmartDashboard.putNumber("PV/Yaw", target.getYaw());
            SmartDashboard.putNumber("PV/Pitch", target.getPitch());
            SmartDashboard.putNumber("PV/Area", target.getArea());
            SmartDashboard.putNumber("PV/Target ID", target.getFiducialId());

        } else {
            // Valores por defecto cuando no hay target
            SmartDashboard.putNumber("PV/Yaw", 0.0);
            SmartDashboard.putNumber("PV/Pitch", 0.0);
            SmartDashboard.putNumber("PV/Area", 0.0);
            SmartDashboard.putNumber("PV/Target ID", -1);
        }

        // Latencia total del pipeline (ms) - unavailable in this PhotonVision version, show 0.0 as fallback
        SmartDashboard.putNumber(
            "PV/Latency (ms)",
            0.0
        );
    }
}
