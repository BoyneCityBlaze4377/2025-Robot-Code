package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface;
import choreo.trajectory.SwerveSample;

import com.fasterxml.jackson.databind.node.BooleanNode;
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.config.PIDConstants;
// import com.pathplanner.lib.config.RobotConfig;
// import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.SensorConstants;
import frc.robot.Constants.AutoAimConstants.ReefStation;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class DriveTrain extends SubsystemBase {
  private int periodicTimer = 1;

  // Robot swerve modules
  private final SwerveModule m_frontLeft, m_frontRight, m_backLeft, m_backRight;

  // The gyro sensor
  public final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);
  private boolean fieldOrientation = true;
  private boolean isLocked = false;
  private boolean slow = false;
  private boolean isBrake = true;
  private double speedScaler, heading, x, y, omega, translationElevatorHeightSpeedScaler, 
                 rotationElevatorHeightSpeedScaler;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry;
  SwerveDrivePoseEstimator m_poseEstimator;

  Field2d estimateField;

  private final GenericEntry elevatorHeight;

  //Choreo stuff
  private final PIDController xController = new PIDController(10.0, 0.0, 0.0);
  private final PIDController yController = new PIDController(10.0, 0.0, 0.0);
  private final PIDController headingController = new PIDController(7.5, 0.0, 0.0);
    
  /** Creates a new DriveTrain. */
  public DriveTrain() {
    m_frontLeft = new SwerveModule("frontLeft", DriveConstants.frontLeftDriveMotorPort, 
                                                     DriveConstants.frontLeftTurningMotorPort,
                                                     DriveConstants.frontLeftTurningEncoderPort, 
                                                     DriveConstants.frontLeftDriveMotorReversed,
                                                     DriveConstants.frontLeftTurningMotorReversed, 
                                                     DriveConstants.frontLeftAnalogEncoderOffset, 
                                                     DriveConstants.frontLeftAbsReversed);

    m_frontRight = new SwerveModule("frontRight", DriveConstants.frontRightDriveMotorPort, 
                                                       DriveConstants.frontRightTurningMotorPort,
                                                       DriveConstants.frontRightTurningEncoderPort, 
                                                       DriveConstants.frontRightDriveMotorReversed,
                                                       DriveConstants.frontRightTurningMotorReversed, 
                                                       DriveConstants.frontRightAnalogEncoderOffset, 
                                                       DriveConstants.frontRightAbsReversed);

    m_backLeft = new SwerveModule("backLeft", DriveConstants.backLeftDriveMotorPort, 
                                                   DriveConstants.backLeftTurningMotorPort,
                                                   DriveConstants.backLeftTurningEncoderPort, 
                                                   DriveConstants.backLeftDriveMotorReversed,
                                                   DriveConstants.backLeftTurningMotorReversed, 
                                                   DriveConstants.backLeftAnalogEncoderOffset, 
                                                   DriveConstants.backLeftAbsReversed);

    m_backRight = new SwerveModule("backRight", DriveConstants.backRightDriveMotorPort, 
                                                   DriveConstants.backRightTurningMotorPort,
                                                   DriveConstants.backRightTurningEncoderPort, 
                                                   DriveConstants.backRightDriveMotorReversed,
                                                   DriveConstants.backRightTurningMotorReversed, 
                                                   DriveConstants.backRightAnalogEncoderOffset, 
                                                   DriveConstants.backRightAbsReversed);

    m_odometry = new SwerveDriveOdometry(DriveConstants.driveKinematics, m_gyro.getRotation2d(),
                                               new SwerveModulePosition[] {m_frontLeft.getPosition(), m_frontRight.getPosition(),
                                                                           m_backLeft.getPosition(), m_backRight.getPosition()});

    m_poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.driveKinematics, m_gyro.getRotation2d(),
    new SwerveModulePosition[] {m_frontLeft.getPosition(), m_frontRight.getPosition(),
                                m_backLeft.getPosition(), m_backRight.getPosition()}, getPose());

    zeroHeading();
    brakeAll();
    fieldOrientation = true;
    isLocked = false;
    speedScaler = DriveConstants.speedScaler;
    slow = false;
    resetEncoders();

    translationElevatorHeightSpeedScaler = 1;
    rotationElevatorHeightSpeedScaler = 1;
    elevatorHeight = IOConstants.DiagnosticTab.add("ElevatorEncoder", 115).getEntry();    

    estimateField = new Field2d();

    headingController.enableContinuousInput(-Math.PI, Math.PI);

    x = 0;
    y = 0;
    omega = 0;
    
  //   // remy code
  //   RobotConfig config = null;

  //   AutoBuilder.configure(
  //   this::getPose,
  //   this::resetOdometry,
  //   this::getChassisSpeeds,//ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
  //   (speeds, feedForwards) ->  autonDrive(speedScaler, speedScaler, heading),
  //   new PPHolonomicDriveController(new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
  //   config,
  //   () -> {
  //     var alliance = DriverStation.getAlliance();
  //     if (alliance.isPresent()) {
  //       return alliance.get() == DriverStation.Alliance.Red;
  //     }
  //     return false;
  //   },
  //   this 
  // );
  }

  @Override
  public void periodic() {
    if (periodicTimer > 10) {
      m_frontLeft.update();
      m_frontRight.update();
      m_backLeft.update();
      m_backRight.update();

      periodicTimer = 0;
    }

    isBrake = m_frontLeft.getIdleMode() == IdleMode.kBrake ? true : false;

    updatePoseEstimator();
    estimateField.setRobotPose(getPoseEstimate());

    /** Dashboard Posting */
    SmartDashboard.putNumber("gyro heading", m_gyro.getAngle());
    SmartDashboard.putNumber("FilteredRobotHeading", heading);
    SmartDashboard.putNumber("pitch", m_gyro.getPitch());
    SmartDashboard.putNumber("roll", m_gyro.getRoll());
    SmartDashboard.putNumber("distance", getAverageDistance());
    SmartDashboard.putString("pose", getPose().toString());
    SmartDashboard.putBoolean("Connected", m_gyro.isConnected());
    SmartDashboard.putBoolean("Calibrating", m_gyro.isCalibrating());
    SmartDashboard.putBoolean("Field Oriented", fieldOrientation);
    SmartDashboard.putBoolean("isLocked", isLocked);
    SmartDashboard.putBoolean("SlowMode", slow);
    SmartDashboard.putNumber("GyroDisplacment", m_gyro.getDisplacementX());
    SmartDashboard.putBoolean("brake mode", isBrake);
    SmartDashboard.putData(estimateField);

    SmartDashboard.putNumber("X Speed", x);
    SmartDashboard.putNumber("y speed", y);
    SmartDashboard.putNumber("omega", omega);

    // Update the odometry in the periodic block
    m_odometry.update(m_gyro.getRotation2d(), getSwerveModulePositions());

    heading = m_gyro.getYaw() - m_gyro.getAngleAdjustment();
    translationElevatorHeightSpeedScaler = DriveConstants.speedScaler - DriveConstants.elevatorHeightFactorTranslation 
                                                                        * elevatorHeight.getDouble(115);
    rotationElevatorHeightSpeedScaler = 1 - DriveConstants.elevatorHeightFactorRotation * elevatorHeight.getDouble(115);

    //drive
    instanceDrive(x * translationElevatorHeightSpeedScaler, y * translationElevatorHeightSpeedScaler, 
                  omega * rotationElevatorHeightSpeedScaler, fieldOrientation);
  }

  private void instanceDrive(double xSpeed, double ySpeed, double omega, boolean fieldRelative) {
    var swerveModuleStates = DriveConstants.driveKinematics.toSwerveModuleStates((fieldOrientation
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omega, m_gyro.getRotation2d()) 
                : new ChassisSpeeds(-xSpeed, -ySpeed, omega)));

    setModuleStates(swerveModuleStates, (xSpeed == 0 && ySpeed == 0 && omega == 0));
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
    omega = rot * Math.PI;
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
    omega = rot * Math.PI;
    fieldOrientation = true;
  }

  public void chassisSpeedDrive(ChassisSpeeds speeds) {
    brakeAll();
    x = speeds.vxMetersPerSecond;
    y = speeds.vyMetersPerSecond;
    omega = speeds.omegaRadiansPerSecond;
  }

//   public void choreoDrive(SwerveSample sample) {
//     // Get the current pose of the robot
//     Pose2d pose = getPose();

//     // Generate the next speeds for the robot
//     ChassisSpeeds speeds = new ChassisSpeeds(
//         sample.vx + xController.calculate(pose.getX(), sample.x),
//         sample.vy + yController.calculate(pose.getY(), sample.y),
//         sample.omega + headingController.calculate(pose.getRotation().getRadians(), sample.heading)
//     );

//     // Apply the generated speeds
//     chassisSpeedDrive(speeds);
//   }

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
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(m_gyro.getRotation2d(),
        new SwerveModulePosition[] {m_frontLeft.getPosition(), m_frontRight.getPosition(),
                                    m_backLeft.getPosition(), m_backRight.getPosition()}, pose);
  }

  public synchronized void updatePoseEstimator() {
    m_poseEstimator.update(m_gyro.getRotation2d(), getSwerveModulePositions());
  }

  public synchronized void setPoseEstimate(Pose2d pose) {
    m_poseEstimator.resetPosition(m_gyro.getRotation2d(), getSwerveModulePositions(), pose);
  }

  public synchronized Pose2d getPoseEstimate() {
    return m_poseEstimator.getEstimatedPosition();
  }

   /** Switches the idle modes of all modlues' drive motors */
   public void switchBrake(){
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
  public void stop(){
    m_frontLeft.stop();
    m_frontRight.stop();
    m_backLeft.stop();
    m_backRight.stop();
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
    fieldOrientation = !fieldOrientation;
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

  /**
   * Sets the offset of the gyro.
   * 
   * @param offsetDegrees The number of degrees to offset by.
   */
  public void setGyroOffset(double offsetDegrees) {
    m_gyro.setAngleAdjustment(offsetDegrees);
  }

  /** Zeroes the heading of the robot */
  public void zeroHeading() {
    m_gyro.zeroYaw();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return The robot's heading in degrees, from -180 to 180
   */
  public synchronized double getHeading() {
    return heading;
  }

   /** @return The roll of the gyro */
   public double getRoll(){
    return m_gyro.getRoll();
  }
  
  /** @return The pitch of the gyro */
  public double getPitch(){
    return m_gyro.getPitch();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second.
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.gyroReversed ? -1.0 : 1.0);
  }

  /** @return The average distance of all modules */
  public double getAverageDistance(){
    double distance = (Math.abs(m_frontLeft.getDistance()) + Math.abs(m_frontRight.getDistance()) +
                       Math.abs(m_backLeft.getDistance()) + Math.abs(m_backRight.getDistance())) / 4;
    return distance;
  }

  public ReefStation getEstimatedStation() {
    double prevError = 180;
    double selectedAngle = 0;

    for (int i = 0; i < AutoAimConstants.reefStationAngles.length; i++) {
      double error = Math.abs(heading - AutoAimConstants.reefStationAngles[i]);
      if (error < prevError){
        prevError = error;
        selectedAngle = AutoAimConstants.reefStationAngles[i];
      }
    }
    return AutoAimConstants.reefStationFromAngle.get(selectedAngle);
  }
}