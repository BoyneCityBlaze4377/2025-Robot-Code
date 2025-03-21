package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import au.grapplerobotics.interfaces.LaserCanInterface;

import java.io.IOException;
import java.util.Map;
import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.Lib.AdvancedPose2D;
import frc.Lib.LimelightHelpers;
import frc.Lib.LimelightHelpers.PoseEstimate;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.Constants.AutoAimConstants.Alignment;
import frc.robot.Constants.AutoAimConstants.ReefStation;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.SensorConstants;

public class DriveTrain extends SubsystemBase {
  private int periodicTimer = 1;

  // Robot swerve modules
  private final SwerveModule m_frontLeft, m_frontRight, m_backLeft, m_backRight;

  // The gyro sensor
  //private final AHRS m_gyro;
  private final Pigeon2 m_gyro;
  private boolean fieldOrientation = true,
                  isLocked = false, slow = false, 
                  isBrake = true, autonInRange = false, useScalers = false, 
                  straightDriveBackwards = false, isBlue = true;
  private double speedScaler, heading, x, y, omega,  elevatorHeight;

  private AprilTagFieldLayout fieldLayout;

  private final AutoAimSubsystem m_autoAimSubsystem;

  // Odometry class for tracking robot pose
  private final SwerveDriveOdometry m_odometry;
  private final SwerveDrivePoseEstimator poseEstimator;

  private final Field2d estimateField;

  private final Elevator m_elevator;

  private final GenericEntry robotHeading, poseEstimate, xSpeedSender, 
                             ySpeedSender, omegaSender, matchTime,
                             desPoseSender, desAlignSender, desStationSender,
                             atDesPose, fmsInfo, orientationSender;

  private final PIDController xController = new PIDController(AutoAimConstants.transkP,
                                                              AutoAimConstants.transkI,
                                                              AutoAimConstants.transkD);
  private final PIDController yController = new PIDController(AutoAimConstants.transkP,
                                                              AutoAimConstants.transkI,
                                                              AutoAimConstants.transkD);
  private final PIDController headingController = new PIDController(AutoAimConstants.turnkP,
                                                                    AutoAimConstants.turnkI,
                                                                    AutoAimConstants.turnkD);

  private AdvancedPose2D initialPose = new AdvancedPose2D();

  // From AAS
  private double tx, ty, ta, tID, dis;
  private final String cameraName;
  private Alliance m_alliance;
  //private final LaserCan laserCan;
  private LaserCanInterface.Measurement lcMeasurement;
  private AdvancedPose2D desiredPose;
  private Alignment desiredAlignment;
  private ReefStation estimatedStation;
    
  /** Creates a new DriveTrain. */
  public DriveTrain(Elevator elevator, AutoAimSubsystem autoAimSubsystem, String limelightName) {
    m_frontLeft = new SwerveModule("frontLeft", ModuleConstants.frontLeftDriveMotorPort, 
                                                     ModuleConstants.frontLeftTurningMotorPort,
                                                     ModuleConstants.frontLeftTurningEncoderPort, 
                                                     ModuleConstants.frontLeftDriveMotorReversed,
                                                     ModuleConstants.frontLeftTurningMotorReversed, 
                                                     ModuleConstants.frontLeftAnalogEncoderOffset, 
                                                     ModuleConstants.frontLeftAbsReversed);

    m_frontRight = new SwerveModule("frontRight", ModuleConstants.frontRightDriveMotorPort, 
                                                       ModuleConstants.frontRightTurningMotorPort,
                                                       ModuleConstants.frontRightTurningEncoderPort, 
                                                       ModuleConstants.frontRightDriveMotorReversed,
                                                       ModuleConstants.frontRightTurningMotorReversed, 
                                                       ModuleConstants.frontRightAnalogEncoderOffset, 
                                                       ModuleConstants.frontRightAbsReversed);

    m_backLeft = new SwerveModule("backLeft", ModuleConstants.backLeftDriveMotorPort, 
                                                   ModuleConstants.backLeftTurningMotorPort,
                                                   ModuleConstants.backLeftTurningEncoderPort, 
                                                   ModuleConstants.backLeftDriveMotorReversed,
                                                   ModuleConstants.backLeftTurningMotorReversed, 
                                                   ModuleConstants.backLeftAnalogEncoderOffset, 
                                                   ModuleConstants.backLeftAbsReversed);

    m_backRight = new SwerveModule("backRight", ModuleConstants.backRightDriveMotorPort, 
                                                     ModuleConstants.backRightTurningMotorPort,
                                                     ModuleConstants.backRightTurningEncoderPort, 
                                                     ModuleConstants.backRightDriveMotorReversed,
                                                     ModuleConstants.backRightTurningMotorReversed, 
                                                     ModuleConstants.backRightAnalogEncoderOffset, 
                                                     ModuleConstants.backRightAbsReversed);

    m_gyro = new Pigeon2(SensorConstants.gyroID);

    estimateField = new Field2d();

    m_autoAimSubsystem = autoAimSubsystem;
    desiredAlignment = Alignment.left;
    desiredPose = new AdvancedPose2D();
    estimatedStation = ReefStation.front;

    m_odometry = new SwerveDriveOdometry(DriveConstants.driveKinematics, m_gyro.getRotation2d(),
                                         getSwerveModulePositions());
    poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.driveKinematics, m_gyro.getRotation2d(), 
                                                 getSwerveModulePositions(), initialPose,
                                                 AutoAimConstants.poseEstimateOdometryStdDev,
                                                 AutoAimConstants.poseEstimateVisionStdDev);
                                                 
    estimateField.setRobotPose(poseEstimator.getEstimatedPosition());

    try {
      fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);
      fieldLayout.setOrigin(new Pose3d());
    } catch (IOException e) {}

    robotHeading = IOConstants.TeleopTab.add("Robot Heading", heading)
                                        .withWidget("Radial Gauge")
                                        .withProperties(Map.of("start_angle", -180, "end_angle", 180,
                                                               "min_value", -180, "max_value", 180,
                                                               "wrap_value", true, "show_pointer", true))
                                        .getEntry();
    poseEstimate = IOConstants.DiagnosticTab.add("Field Position", estimateField.getRobotPose())
                                            .withWidget("Field")
                                            .withProperties(Map.of("robot_width", DriveConstants.trackWidth,
                                                                   "robot_length", DriveConstants.wheelBase))
                                            .getEntry();
    xSpeedSender = IOConstants.TeleopTab.add("xSpeed", 0)
                                        .withWidget("Number Slider")
                                        .withProperties(Map.of("min_value", -1, "max_value", 1))
                                        .getEntry();
    ySpeedSender = IOConstants.TeleopTab.add("ySpeed", 0)
                                        .withWidget("Number Slider")
                                        .withProperties(Map.of("min_value", -1, "max_value", 1))
                                        .getEntry();
    omegaSender = IOConstants.TeleopTab.add("rot", 0)
                                       .withWidget("Number Slider")
                                       .withProperties(Map.of("min_value", -1, "max_value", 1))
                                       .getEntry();
    matchTime = IOConstants.TeleopTab.add("Match Time", 15)
                                     .withWidget("Match Time")
                                     .withProperties(Map.of("red_start_time", 10, "yellow_start_time", 20))
                                     .getEntry();
    desPoseSender = IOConstants.DiagnosticTab.add("Desired Pose", initialPose.toString())
                                             .withWidget("Text Display")
                                             .getEntry();
    desAlignSender = IOConstants.DiagnosticTab.add("Desired Alignment", desiredAlignment.toString())
                                              .withWidget("Text Display")
                                              .getEntry();
    desStationSender = IOConstants.DiagnosticTab.add("Estimated Reefstation", estimatedStation.toString())
                                                .withWidget("Text Display")
                                                .getEntry();
    atDesPose = IOConstants.TeleopTab.add("At Desired Pose", false)
                                     .withWidget("Boolean Box")
                                     .getEntry();
    fmsInfo = IOConstants.ConfigTab.add("FMSInfo", 0)
                                   .withWidget("FMSInfo")
                                   .getEntry();
    orientationSender = IOConstants.TeleopTab.add("Field Oriented?", true)
                                             .withWidget("Boolean Box")
                                             .getEntry();

    zeroHeading();
    brakeAll();
    fieldOrientation = true;
    isLocked = false;
    useScalers = false;
    speedScaler = DriveConstants.speedScaler;
    slow = false;
    resetEncoders();

    m_elevator = elevator;
    elevatorHeight = m_elevator.getEncoderVal();

    m_gyro.reset();

    xController.setTolerance(AutoAimConstants.transkTolerance);
    yController.setTolerance(AutoAimConstants.transkTolerance);
    headingController.setTolerance(AutoAimConstants.turnkTolerance);
    headingController.enableContinuousInput(-Math.PI, Math.PI);

    x = 0;
    y = 0;
    omega = 0;

    //From AAS
    cameraName = limelightName;
    m_alliance = Alliance.Blue;
    // laserCan = new LaserCan(17);

    // try {laserCan.setRangingMode(RangingMode.SHORT);} catch (ConfigurationFailedException e) {}

    // lcMeasurement = laserCan.getMeasurement();
    // dis = -1;

    LimelightHelpers.SetRobotOrientation(cameraName, initialPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("camera_robotspace_set")
    //                 .setDoubleArray(SensorConstants.limelightRobotSpacePose);

    SmartDashboard.putData("Swerve Drive", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");

        builder.addDoubleProperty("Front Left Angle", () -> m_frontLeft.getState().angle.getRadians(), null);
        builder.addDoubleProperty("Front Left Velocity", () -> m_frontLeft.getState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Front Right Angle", () -> m_frontRight.getState().angle.getRadians(), null);
        builder.addDoubleProperty("Front Right Velocity", () -> m_frontRight.getState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Back Left Angle", () -> m_backLeft.getState().angle.getRadians(), null);
        builder.addDoubleProperty("Back Left Velocity", () -> m_backLeft.getState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Back Right Angle", () -> m_backRight.getState().angle.getRadians(), null);
        builder.addDoubleProperty("Back Right Velocity", () -> m_backRight.getState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Robot Angle", () -> m_gyro.getRotation2d().getRadians(), null);
      }
    });
  }

  @Override
  public void periodic() {
    if (periodicTimer >= 10) {
      m_frontLeft.update();
      m_frontRight.update();
      m_backLeft.update();
      m_backRight.update();

      periodicTimer = 0;
    }

    heading = m_gyro.getYaw().getValueAsDouble();
    if (heading > 180) {
      m_gyro.setYaw(heading - 360);
    } else if (heading <= -180) {
      m_gyro.setYaw(heading + 360);
    }

    isBrake = m_frontLeft.getIdleMode() == IdleMode.kBrake;

    updateOdometry();
    estimateField.setRobotPose(poseEstimator.getEstimatedPosition());
    elevatorHeight = m_elevator.getEncoderVal();

    /** Dashboard Posting */
    robotHeading.setDouble(heading);
    desPoseSender.setString(desiredPose.toString());
    desAlignSender.setString(desiredAlignment.toString());
    desStationSender.setString(estimatedStation.toString());
    atDesPose.setBoolean(atSetpoints());
    matchTime.setDouble(DriverStation.getMatchTime());
    orientationSender.setBoolean(fieldOrientation);
    poseEstimate.setValue(estimateField.getRobotPose());

    // Update the odometry in the periodic block
    m_odometry.update(m_gyro.getRotation2d(), getSwerveModulePositions());
    poseEstimator.update(m_gyro.getRotation2d(), getSwerveModulePositions());
    if (getPoseEstimate().isPresent()) {
      poseEstimator.addVisionMeasurement(getPoseEstimate().get().pose, 
                                         getPoseEstimate().get().timestampSeconds);
    }

    poseEstimate.setValue(poseEstimator.getEstimatedPosition().toString());

    //drive
    rawDrive(x , y, omega, fieldOrientation, useScalers);

    periodicTimer ++;

    // SmartDashboard.putBoolean("INRANGE", autonInRange);
    // SmartDashboard.putBoolean("Orientation", fieldOrientation);
    // SmartDashboard.putBoolean("NAVX", m_gyro.isConnected());
    // SmartDashboard.putBoolean("CAlib", m_gyro.isCalibrating());

    heading = m_gyro.getYaw().getValueAsDouble();
    if (heading > 180) {
      m_gyro.setYaw(heading - 360);
    } else if (heading <= -180) {
      m_gyro.setYaw(heading + 360);
    }

    autonInRange = Math.hypot(xController.getError(), yController.getError()) <= AutonConstants.inRangeThreshold;

    switch (m_elevator.getCurrentPosition()) {
      case processor:
        straightDriveBackwards = true;
        break;
      case L2:
        straightDriveBackwards = false;
        break;
      case L2algae:
        straightDriveBackwards = true;
        break;
      case L3:
        straightDriveBackwards = false;
        break;
      case L3algae:
        straightDriveBackwards = true;
        break;
      case L4:
        straightDriveBackwards = false;
        break;
      case HP:
        straightDriveBackwards = false;
        break;
      default:
        straightDriveBackwards = false;
        break;
    }

    //From AAS
    /* LIMELIGHT */
    NetworkTable table = NetworkTableInstance.getDefault().getTable(SensorConstants.limeLightName);
    tx = table.getEntry("tx").getDouble(0);
    ty = table.getEntry("ty").getDouble(0);
    ta = table.getEntry("ta").getDouble(0);
    tID = table.getEntry("fID").getDouble(0);

    /* LaserCAN */
    // lcMeasurement = laserCan.getMeasurement();
    // dis = lcMeasurement != null && lcMeasurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT 
    //       ? lcMeasurement.distance_mm : -1;

    estimatedStation = isBlue ? AutoAimConstants.blueReefStationFromAngle.get(getEstimatedStationAngle()) 
                              : AutoAimConstants.redReefStationFromAngle.get(getEstimatedStationAngle());
    desiredPose = isBlue ? AutoAimConstants.blueReef.get(estimatedStation) 
                         : AutoAimConstants.redReef.get(estimatedStation);

    LimelightHelpers.SetRobotOrientation(cameraName, heading, 0, 0, 0, 0, 0);
  }

  private void rawDrive(double xSpeed, double ySpeed, double omega, boolean fieldRelative, boolean scale) {
    if (scale) {
      xSpeed *= calcTransHeightScaler(elevatorHeight);
      ySpeed *= calcTransHeightScaler(elevatorHeight);
      omega *= calcRotHeightScaler(elevatorHeight);
    }

    var swerveModuleStates = DriveConstants.driveKinematics.toSwerveModuleStates(fieldOrientation
                           ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omega, m_gyro.getRotation2d()) 
                           : new ChassisSpeeds(xSpeed, ySpeed, omega));

    setModuleStates(swerveModuleStates, (xSpeed == 0 && ySpeed == 0 && omega == 0));

    xSpeedSender.setDouble(xSpeed);
    ySpeedSender.setDouble(ySpeed);
    omegaSender.setDouble(omega);
  }

  public void straightDrive(double xSpeed) {
    x = xSpeed * (straightDriveBackwards ? -1 : 1);
    y = 0;
    omega = 0;
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   */
  public void teleopDrive(double xSpeed, double ySpeed, double rot) {
    rot = Math.pow(rot, 3);

    x = xSpeed * DriveConstants.maxSpeedMetersPerSecond * speedScaler;
    y = ySpeed * DriveConstants.maxSpeedMetersPerSecond * speedScaler;
    omega = rot * DriveConstants.maxRotationSpeedRadiansPerSecond * speedScaler;

    useScalers = true;
  }

  /**
   * Drive the robot autonomously.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   */
  public void autonDrive(double xSpeed, double ySpeed, double rot) {
    brakeAll();
    x = xSpeed;
    y = ySpeed;
    omega = rot;
    fieldOrientation = true;
    useScalers = true;
  }

  public void chassisSpeedDrive(ChassisSpeeds speeds) {
    brakeAll();
    x = speeds.vxMetersPerSecond;
    y = speeds.vyMetersPerSecond;
    omega = speeds.omegaRadiansPerSecond;
    useScalers = true;
  }

  public void setPIDSetpoints(double xSetpoint, double ySetpoint, double headingSetpoint) {
    xController.setSetpoint(xSetpoint);
    yController.setSetpoint(ySetpoint);
    headingController.setSetpoint(headingSetpoint);
  }

  public void PIDDrive() {
    x = MathUtil.clamp(xController.calculate(getPose().getX()), -AutoAimConstants.maxPIDDriveSpeed, 
                                                                 AutoAimConstants.maxPIDDriveSpeed);
    y = MathUtil.clamp(yController.calculate(getPose().getY()), -AutoAimConstants.maxPIDDriveSpeed, 
                                                                 AutoAimConstants.maxPIDDriveSpeed);
    omega = MathUtil.clamp(headingController.calculate(getPose().getRotation().getRadians()), 
                                                      -AutoAimConstants.maxPIDRot, 
                                                       AutoAimConstants.maxPIDRot);

    useScalers = false;
  }

  public boolean atSetpoints() {
    return xController.atSetpoint() && yController.atSetpoint() && headingController.atSetpoint();
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates, boolean isNeutral) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.maxSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(desiredStates[0], isNeutral);
    m_frontRight.setDesiredState(desiredStates[1], isNeutral);
    m_backLeft.setDesiredState(desiredStates[2], isNeutral);
    m_backRight.setDesiredState(desiredStates[3], isNeutral);
  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    return new SwerveModulePosition[] {m_frontLeft.getPosition(), m_frontRight.getPosition(),
                                       m_backLeft.getPosition(), m_backRight.getPosition()};
  }

  /**
   * Sets the positions of the SwerveModules' angle motors.
   *  
   * @param desiredStates The state containing the desired angle for the angle motor;
   */
  public void setModuleAngles(SwerveModuleState[] desiredStates) {
    m_frontLeft.setAngle(desiredStates[0], false);
    m_frontRight.setAngle(desiredStates[1], false);
    m_backLeft.setAngle(desiredStates[2], false);
    m_backRight.setAngle(desiredStates[3], false);
  }

  /** 
   * Sets the pose of the robot to be locked: all modules' angles form an X
   */
  public void lockPose() {
    m_frontLeft.setLockedState(new SwerveModuleState(0, Rotation2d.fromDegrees(225)));
    m_frontRight.setLockedState(new SwerveModuleState(0, Rotation2d.fromDegrees(315)));
    m_backLeft.setLockedState(new SwerveModuleState(0, Rotation2d.fromDegrees(315)));
    m_backRight.setLockedState(new SwerveModuleState(0, Rotation2d.fromDegrees(225)));
  }

  public ChassisSpeeds getChassisSpeeds() {
    return fieldOrientation ? ChassisSpeeds.fromFieldRelativeSpeeds(x, y, omega, m_gyro.getRotation2d()) 
                            : new ChassisSpeeds(-x, -y, omega);
  }

  /** 
   * Sets whether or not the DriveTrain is in the Locked Pose. 
   *
   * @param locked Whether or not the robot is in the Locked Pose.
   */
  public void setLocked(boolean locked) {
    isLocked = locked;
  }

  /**
   * Scales the max speed of the robot.
   * 
   * @param scaler What to multiply the SpeedScaler by.
   */
  public void scaleSpeedScaler(double scaler) {
    speedScaler *= scaler;
  }

  /**@return The Drivetrain's current SpeedScaler */
  public synchronized double getSpeedScaler() {
    return speedScaler;
  }

  /**
   * Sets whether or not the robot is in SlowMode.
   * 
   * @param isSlow True for slow, false for regular.
   */
  public void setIsSlow(boolean isSlow) {
    slow = isSlow;
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public synchronized Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
    //return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void setOdometry(Pose2d pose) {
    m_odometry.resetPosition(m_gyro.getRotation2d(), getSwerveModulePositions(), pose);
  }

  private void updateOdometry() {
    m_odometry.update(m_gyro.getRotation2d(), getSwerveModulePositions());
  }

   /** Switches the idle modes of all modlues' drive motors */
   public void switchBrake() {
    if (isBrake) {
      coastAll();
    } else {
      brakeAll();
    }
  }

  /** Sets all idle modes to Brake */
  public void brakeAll() {
    m_frontLeft.brake();
    m_frontRight.brake();
    m_backLeft.brake();
    m_backRight.brake();
  }

  /** Sets all idle modes to Coast */
  public void coastAll() {
    m_frontLeft.coast();
    m_frontRight.coast();
    m_backLeft.coast();
    m_backRight.coast();
  }

  /** @return The current IdleMode of the DriveTrain */
  public synchronized IdleMode getIdleMode() {
    return m_frontLeft.getIdleMode();
  }

  /**
   * Sets the IdleMode of the DriveTrain.
   * 
   * @param mode The desired IdleMode for the DriveTrain.
   */
  public synchronized void setIdleMode(IdleMode mode) {
    m_frontLeft.setIdleMode(mode);
    m_frontRight.setIdleMode(mode);
    m_backLeft.setIdleMode(mode);
    m_backRight.setIdleMode(mode);
  }

  /** Stops drive motors for all modules */
  public void stop() {
    x = 0;
    y = 0;
    omega = 0;
  }

  /** Reset encoders of all modules */
  public synchronized void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_backLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_backRight.resetEncoders();
  }

  /** Switches whether or not the robot drives field oriented */
  public synchronized void switchOrientation() {
    if (fieldOrientation) {
      fieldOrientation = false;
    } else {
      fieldOrientation = true;
    }
  }

  /**
   * Sets whether or not the robot should drive Field Oriented.
   * 
   * @param isFieldOriented True for Field Oriented, false for Robot Oriented.
   */
  public synchronized void setOrientation(boolean isFieldOriented) {
    fieldOrientation = isFieldOriented;
  }

  public synchronized boolean isFieldOriented() {
    return fieldOrientation;
  }

  // /**
  //  * Sets the offset of the gyro.
  //  * 
  //  * @param offsetDegrees The number of degrees to offset by.
  //  */
  // public void setGyroOffset(double offsetDegrees) {
  //   m_gyro.setAngleAdjustment(offsetDegrees);
  // }

  /** Zeroes the heading of the robot */
  public void zeroHeading() {
    m_gyro.setYaw(0);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return The robot's heading in degrees, from -180 to 180
   */
  public synchronized double getHeading() {
    return heading;
  }

  public synchronized double getRawHeading() {
    return m_gyro.getYaw().getValueAsDouble();
  }

  //  /** @return The roll of the gyro */
  //  public double getRoll(){
  //   return m_gyro.getRoll();
  // }
  
  // /** @return The pitch of the gyro */
  // public double getPitch(){
  //   return m_gyro.getPitch();
  // }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second.
   */
  public double getTurnRate() {
    return m_gyro.getAngularVelocityZWorld().getValueAsDouble() * (DriveConstants.gyroReversed ? -1.0 : 1.0);
  }

  /** @return The average distance of all modules */
  public double getAverageDistance(){
    return (Math.abs(m_frontLeft.getDistance()) + Math.abs(m_frontRight.getDistance()) +
            Math.abs(m_backLeft.getDistance()) + Math.abs(m_backRight.getDistance())) / 4;
    
  }

  public boolean getInRange() {
    return autonInRange;
  }

  public void setInRange(boolean isInRange) {
    autonInRange = isInRange;
  }

  public double calcTransHeightScaler(double height) {
    return ((DriveConstants.minDriveSpeed - DriveConstants.maxDriveSpeed) / 
             Math.pow(ElevatorConstants.upperLimit - ElevatorConstants.lowerLimit, 2)) * 
             Math.pow(height - ElevatorConstants.lowerLimit, 2) + DriveConstants.maxDriveSpeed;
  }

  public double calcRotHeightScaler(double height) {
    return ((DriveConstants.minRotSpeed - DriveConstants.maxRotSpeed) / 
             Math.pow(ElevatorConstants.upperLimit - ElevatorConstants.lowerLimit, 2)) * 
             Math.pow(height - ElevatorConstants.lowerLimit, 2) + DriveConstants.maxRotSpeed;
  }

  public double getEstimatedStationAngle() {
    double prevError = 180;
    double selectedAngle = 0;

    for (int i = 0; i < AutoAimConstants.reefStationAngles.length; i++) {
      double error = Math.abs(getHeading() - AutoAimConstants.reefStationAngles[i]);
      if (error < prevError){
        prevError = error;
        selectedAngle = AutoAimConstants.reefStationAngles[i];
      }
    }
    return selectedAngle;
  }


  //FromAAS
  private Optional<Pose2d> getEstimatedPose2dBlue() {
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);
    return Optional.of(new Pose3d(new Translation3d(mt2.pose.getTranslation()), new Rotation3d(mt2.pose.getRotation())).toPose2d());
  }

  private Optional<PoseEstimate> getPoseEstimateBlue() {
    return Optional.of(LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName));
  }

  private Optional<Pose2d> getEstimatedPose2dRed() {
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiRed(cameraName);
    return Optional.of(new Pose3d(new Translation3d(mt2.pose.getTranslation()), new Rotation3d(mt2.pose.getRotation())).toPose2d());
  }

  private Optional<PoseEstimate> getPoseEstimateRed() {
    return Optional.of(LimelightHelpers.getBotPoseEstimate_wpiRed(cameraName));
  }

  public Optional<Pose2d> getEstimatedPose2d() {
    return isBlue ? getEstimatedPose2dBlue() : getEstimatedPose2dRed();
  }

  public Optional<PoseEstimate> getPoseEstimate() {
    return isBlue ? getPoseEstimateBlue() : getPoseEstimateRed();
  }

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

  public synchronized void setInitialPose(AdvancedPose2D pose) {
    poseEstimator.resetPose(pose);
    initialPose = pose;
  }

  public synchronized AdvancedPose2D getAutoAimPose() {
    return desiredPose.withReefAlignment(desiredAlignment, false);
  }

  public synchronized ReefStation getDesiredStation() {
    return estimatedStation;
  }

  public synchronized void setAlliance(Alliance alliance) {
    m_alliance = alliance;
    isBlue = m_alliance == Alliance.Blue;
  }

  public synchronized Alliance getAlliance() {
    return m_alliance;
  }
}