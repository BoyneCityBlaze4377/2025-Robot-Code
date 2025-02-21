package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SensorConstants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
// import org.photonvision.EstimatedRobotPose;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface;

public class VisionSubsystem extends SubsystemBase {
  private double tx, ty, ta, tID;
  private final String name;
  private final LaserCan laserCan;
  private LaserCanInterface.Measurement lcMeasurement;

  /** Creates a new Vision. */
  public VisionSubsystem(String cameraName) {
    name = cameraName;
    laserCan = new LaserCan(17);
  }

  @Override
  public void periodic() {
    /* LIMELIGHT */
    NetworkTable table = NetworkTableInstance.getDefault().getTable(SensorConstants.limeLightName);
    tx = table.getEntry("tx").getDouble(0);
    ty = table.getEntry("ty").getDouble(0);
    ta = table.getEntry("ta").getDouble(0);

    LimelightTarget_Fiducial[] targets = LimelightHelpers.getLatestResults(SensorConstants.limeLightName)
                                                         .targetingResults.targets_Fiducials;
    tID = (targets.length > 0 ? targets[0].fiducialID : 0);

    SmartDashboard.putNumber("AprilTagID", tID);
    SmartDashboard.putNumber("TX", tx);
    SmartDashboard.putNumber("TY", ty);

    /* LASERCAN */
    lcMeasurement = laserCan.getMeasurement();
    if (lcMeasurement != null && lcMeasurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      SmartDashboard.putNumber("LCMeasurement", laserCan.getMeasurement().distance_mm);
    } else {
      SmartDashboard.putNumber("LCMeasurement", -1);
    }

    SmartDashboard.putBoolean("LCHasMeasurement", lcMeasurement != null && lcMeasurement.status == 
                                                                               LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT);
  }

  // public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
  //   LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
  //   return Optional.of(new EstimatedRobotPose(new Pose3d(new Translation3d(mt2.pose.getTranslation()),
  //                                                         new Rotation3d(mt2.pose.getRotation())),
  //                                             mt2.timestampSeconds,
  //                                             null,
  //                                             null));
  // }

  public Optional<Pose2d> getEstimatedPose2d() {
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    return Optional.of(new Pose3d(new Translation3d(mt2.pose.getTranslation()), new Rotation3d(mt2.pose.getRotation())).toPose2d());
  }

  public double getDistanceMeasurementmm() {
    return lcMeasurement.distance_mm;
  }

  public double getTX() {
    return tx;
  }

  public double getTY() {
    return ty;
  }

  public double getTA() {
    return ta;
  }

  public double getTargetID() {
    return tID;
  }
}
