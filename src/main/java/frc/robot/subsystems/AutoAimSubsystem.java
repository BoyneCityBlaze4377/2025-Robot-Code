package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Lib.AdvancedPose2D;
import frc.Lib.LimelightHelpers;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.SensorConstants;
import frc.robot.Constants.AutoAimConstants.Alignment;
import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;

public class AutoAimSubsystem extends SubsystemBase {
  private double tx, ty, ta, tID, dis;
  private final String name;
  private final LaserCan laserCan;
  private LaserCanInterface.Measurement lcMeasurement;
  private final GenericEntry txSender, targetIDSender, LCMeasurementSender, LCHasMeasurement;
  private AdvancedPose2D desiredPose;
  private Alignment desiredAlignment;

  /** Creates a new Vision. */
  public AutoAimSubsystem(String cameraName) {
    name = cameraName;
    laserCan = new LaserCan(17);
    try {
      laserCan.setRangingMode(RangingMode.SHORT);
    } catch (ConfigurationFailedException e) {}
    lcMeasurement = laserCan.getMeasurement();
    dis = -1;

    txSender = IOConstants.DiagnosticTab.add("tx", tx)
                                        .withWidget("Text Display").getEntry();
    targetIDSender = IOConstants.DiagnosticTab.add("target ID", tID)
                                              .withWidget("Text Display").getEntry();
    LCMeasurementSender = IOConstants.DiagnosticTab.add("LaserCAN measured distance", -1)
                                                   .withWidget("Text Display").getEntry();
    LCHasMeasurement = IOConstants.DiagnosticTab.add("LCHasMeasurement", false)
                                                .withWidget("Boolean Box").getEntry();

    desiredPose = new AdvancedPose2D();
    desiredAlignment = Alignment.left;
  }

  @Override
  public void periodic() {
    /* LIMELIGHT */
    NetworkTable table = NetworkTableInstance.getDefault().getTable(SensorConstants.limeLightName);
    tx = table.getEntry("tx").getDouble(0);
    ty = table.getEntry("ty").getDouble(0);
    ta = table.getEntry("ta").getDouble(0);
    tID = table.getEntry("fID").getDouble(0);

    /* LaserCAN */
    lcMeasurement = laserCan.getMeasurement();
    dis = lcMeasurement != null && lcMeasurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT 
          ? lcMeasurement.distance_mm : -1;

    /* Value Posting */
    txSender.setDouble(getTX());
    targetIDSender.setDouble(getTargetID());
    LCMeasurementSender.setDouble(getDistanceMeasurementmm());
    LCHasMeasurement.setBoolean(lcMeasurement != null && lcMeasurement.status == 
                                                         LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT);

    SmartDashboard.putNumber("TempMeasure", (dis / 1000) - AutoAimConstants.LCToBumperEdgeOffsetMeters);
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    return Optional.of(new EstimatedRobotPose(new Pose3d(new Translation3d(mt2.pose.getTranslation()),
                                                          new Rotation3d(mt2.pose.getRotation())),
                                              mt2.timestampSeconds,
                                              null,
                                              null));
  }

  public Optional<Pose2d> getEstimatedPose2d() {
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    return Optional.of(new Pose3d(new Translation3d(mt2.pose.getTranslation()), new Rotation3d(mt2.pose.getRotation())).toPose2d());
  }

  public double getDistanceMeasurementmm() {
    return lcMeasurement != null && lcMeasurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT 
                                    ? (((dis / 1000) - AutoAimConstants.LCToBumperEdgeOffsetMeters) * 1000) 
                                    : -1;
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

  public void setDesiredPose(AdvancedPose2D pose) {
    desiredPose = pose;
  }

  public void setDesiredAlignment(Alignment alignment) {
    desiredAlignment = alignment;
  }

  public AdvancedPose2D getDesiredPose() {
    return desiredPose;
  }

  public Alignment getDesiredAlignment() {
    return desiredAlignment;
  }
}
