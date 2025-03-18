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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Lib.AdvancedPose2D;
import frc.Lib.LimelightHelpers;
import frc.Lib.LimelightHelpers.PoseEstimate;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.SensorConstants;
import frc.robot.Constants.AutoAimConstants.Alignment;
import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;

public class AutoAimSubsystem extends SubsystemBase {
  private double tx, ty, ta, tID, dis, timeStamp;
  private final String name;
  // private final Alliance m_alliance;
  // private final Joystick stick;
  //private final LaserCan laserCan;
  private LaserCanInterface.Measurement lcMeasurement;
  private final GenericEntry txSender, targetIDSender, LCMeasurementSender, LCHasMeasurement;
  private AdvancedPose2D desiredPose;
  private Alignment desiredAlignment;

  /** Creates a new Vision. */
  public AutoAimSubsystem(String cameraName) {
    name = cameraName;
    // m_alliance = alliance;
    // stick = driverStick;
    // laserCan = new LaserCan(17);

    // try {laserCan.setRangingMode(RangingMode.SHORT);} catch (ConfigurationFailedException e) {}

    // lcMeasurement = laserCan.getMeasurement();
    // dis = -1;

    //LimelightHelpers.SetRobotOrientation(cameraName, initialPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camera_robotspace_set")
                    .setDoubleArray(SensorConstants.limelightRobotSpacePose);

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
    timeStamp = table.getEntry("ts").getDouble(0);

    /* LaserCAN */
    // lcMeasurement = laserCan.getMeasurement();
    // dis = lcMeasurement != null && lcMeasurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT 
    //       ? lcMeasurement.distance_mm : -1;

    /* Value Posting */
    txSender.setDouble(getTX());
    targetIDSender.setDouble(getTargetID());
    // LCMeasurementSender.setDouble(getDistanceMeasurementmm());
    // LCHasMeasurement.setBoolean(lcMeasurement != null && lcMeasurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT);

    SmartDashboard.putString("DesiredPose", desiredPose.toString());
    SmartDashboard.putString("DesiredAlignment", desiredAlignment.toString());

    // switch (stick.getPOV()) {
    //   case 90:
    //     desiredAlignment = Alignment.right;
    //     break;
    //   case 270:
    //     desiredAlignment = Alignment.left;
    //     break;
    //   case -1:
    //     desiredAlignment = Alignment.center;
    //     break;
    //   default:
    //     desiredAlignment = Alignment.center;
    //     break;
    // }
  }

  private Optional<EstimatedRobotPose> getEstimatedGlobalPoseBlue() {
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    return Optional.of(new EstimatedRobotPose(new Pose3d(new Translation3d(mt2.pose.getTranslation()),
                                                          new Rotation3d(mt2.pose.getRotation())),
                                              mt2.timestampSeconds,
                                              null,
                                              null));
  }

  private Optional<Pose2d> getEstimatedPose2dBlue() {
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    return Optional.of(new Pose3d(new Translation3d(mt2.pose.getTranslation()), new Rotation3d(mt2.pose.getRotation())).toPose2d());
  }

  private Optional<PoseEstimate> getPoseEstimateBlue() {
    return Optional.of(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name));
  }

  private Optional<EstimatedRobotPose> getEstimatedGlobalPoseRed() {
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(name);
    return Optional.of(new EstimatedRobotPose(new Pose3d(new Translation3d(mt2.pose.getTranslation()),
                                                          new Rotation3d(mt2.pose.getRotation())),
                                              mt2.timestampSeconds,
                                              null,
                                              null));
  }

  private Optional<Pose2d> getEstimatedPose2dRed() {
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(name);
    return Optional.of(new Pose3d(new Translation3d(mt2.pose.getTranslation()), new Rotation3d(mt2.pose.getRotation())).toPose2d());
  }

  private Optional<PoseEstimate> getPoseEstimateRed() {
    return Optional.of(LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(name));
  }

  // public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
  //   return m_alliance == Alliance.Blue ? getEstimatedGlobalPoseBlue() : getEstimatedGlobalPoseRed();
  // }

  // public Optional<Pose2d> getEstimatedPose2d() {
  //   return m_alliance == Alliance.Blue ? getEstimatedPose2dBlue() : getEstimatedPose2dRed();
  // }

  // public Optional<PoseEstimate> getPoseEstimate() {
  //   return m_alliance == Alliance.Blue ? getPoseEstimateBlue() : getPoseEstimateRed();
  // }

  // public double getDistanceMeasurementmm() {
  //   return lcMeasurement != null && lcMeasurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT 
  //                                   ? (((dis / 1000) - AutoAimConstants.LCToBumperEdgeOffsetMeters) * 1000) 
  //                                   : -1;
  // }

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

  public synchronized void setDesiredPose(AdvancedPose2D pose) {
    desiredPose = pose;
  }

  public synchronized void setDesiredAlignment(Alignment alignment) {
    desiredAlignment = alignment;
  }

  public synchronized AdvancedPose2D getDesiredPose() {
    return desiredPose;
  }

  public synchronized Alignment getDesiredAlignment() {
    return desiredAlignment;
  }
}
