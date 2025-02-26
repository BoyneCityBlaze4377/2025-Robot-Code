package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import java.util.Map;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.AutoAimConstants.ReefStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

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
  private boolean autonInRange = false;
  private double speedScaler, heading, x, y, omega, translationElevatorHeightSpeedScaler, 
                 rotationElevatorHeightSpeedScaler, elevatorHeight;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry;
  SwerveDrivePoseEstimator m_poseEstimator;

  Field2d estimateField;

  private final Elevator m_elevator;

  private final GenericEntry robotHeading, poseEstimate, xSpeedSender, 
                             ySpeedSender, omegaSender;


  //Choreo stuff
  // private final PIDController xController = new PIDController(10.0, 0.0, 0.0);
  // private final PIDController yController = new PIDController(10.0, 0.0, 0.0);
  // private final PIDController headingController = new PIDController(7.5, 0.0, 0.0);
    
  /** Creates a new DriveTrain. */
  public DriveTrain(Elevator elevator) {
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
                                                  m_backLeft.getPosition(), m_backRight.getPosition()}, 
                                                  getPose());

    robotHeading = IOConstants.TeleopTab.add("Robot Heading", heading)
                                       .withWidget("Radial Gauge")
                                       .withProperties(Map.of("start_angle", -180, "end_angle", 180,
                                                              "min_value", -180, "max_value", 180,
                                                              "wrap_value", true, "show_pointer", false))
                                       .getEntry();
    poseEstimate = IOConstants.DiagnosticTab.add("Field", m_poseEstimator.getEstimatedPosition().toString())
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
    SmartDashboard.putData("Swerve", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");
  
        builder.addDoubleProperty("Front Left Angle", () -> m_frontLeft.getState().angle.getRadians(), null);
        builder.addDoubleProperty("Front Left Velocity", () -> m_frontLeft.getState().speedMetersPerSecond, null);
  
        builder.addDoubleProperty("Front Right Angle", () -> m_frontLeft.getState().angle.getRadians(), null);
        builder.addDoubleProperty("Front Right Velocity", () -> m_frontLeft.getState().speedMetersPerSecond, null);
  
        builder.addDoubleProperty("Back Left Angle", () -> m_frontLeft.getState().angle.getRadians(), null);
        builder.addDoubleProperty("Back Left Velocity", () -> m_frontLeft.getState().speedMetersPerSecond, null);
  
        builder.addDoubleProperty("Back Right Angle", () -> m_frontLeft.getState().angle.getRadians(), null);
        builder.addDoubleProperty("Back Right Velocity", () -> m_frontLeft.getState().speedMetersPerSecond, null);
  
        builder.addDoubleProperty("Robot Angle", () -> m_gyro.getRotation2d().getRadians(), null);
      }
    });

    zeroHeading();
    brakeAll();
    fieldOrientation = true;
    isLocked = false;
    speedScaler = DriveConstants.speedScaler;
    slow = false;
    resetEncoders();

    translationElevatorHeightSpeedScaler = 1;
    rotationElevatorHeightSpeedScaler = 1;
    m_elevator = elevator;
    elevatorHeight = m_elevator.getEncoderVal();

    estimateField = new Field2d();

    m_gyro.reset();

    //headingController.enableContinuousInput(-Math.PI, Math.PI);

    x = 0;
    y = 0;
    omega = 0;
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

    isBrake = m_frontLeft.getIdleMode() == IdleMode.kBrake;

    updatePoseEstimator();
    estimateField.setRobotPose(getPoseEstimate());
    elevatorHeight = m_elevator.getEncoderVal();

    /** Dashboard Posting */
    robotHeading.setDouble(heading);
    poseEstimate.setString(getPoseEstimate().toString());
    xSpeedSender.setDouble(x);
    ySpeedSender.setDouble(y);
    omegaSender.setDouble(omega);

    // Update the odometry in the periodic block
    m_odometry.update(m_gyro.getRotation2d(), getSwerveModulePositions());

    heading = m_gyro.getYaw() - m_gyro.getAngleAdjustment();

    translationElevatorHeightSpeedScaler = 1;
                                          //  DriveConstants.maxDriveSpeed 
                                          //  - DriveConstants.elevatorHeightFactorTranslation 
                                          //  * elevatorHeight;
    rotationElevatorHeightSpeedScaler = 1;
                                        // DriveConstants.maxRotspeed 
                                        // - DriveConstants.elevatorHeightFactorRotation 
                                        // * elevatorHeight;

    //drive
    instanceDrive(x * translationElevatorHeightSpeedScaler, y * translationElevatorHeightSpeedScaler, 
                  omega * rotationElevatorHeightSpeedScaler, fieldOrientation);

    periodicTimer ++;
  }

  private void instanceDrive(double xSpeed, double ySpeed, double omega, boolean fieldRelative) {
    var swerveModuleStates = DriveConstants.driveKinematics.toSwerveModuleStates(fieldOrientation
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omega, m_gyro.getRotation2d()) 
                : new ChassisSpeeds(xSpeed, ySpeed, omega));

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

    rot = MathUtil.applyDeadband(rot, .1);

    x = xSpeed * DriveConstants.maxSpeedMetersPerSecond * speedScaler;
    y = ySpeed * DriveConstants.maxSpeedMetersPerSecond * speedScaler;
    omega = rot * Math.PI * 2;
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
  }

  public void chassisSpeedDrive(ChassisSpeeds speeds) {
    brakeAll();
    x = speeds.vxMetersPerSecond;
    y = speeds.vyMetersPerSecond;
    omega = speeds.omegaRadiansPerSecond;
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
    return m_odometry.getPoseMeters();
    //m_poseEstimator.getEstimatedPosition();
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
  public void stop(){
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

  public boolean getInRange() {
    return autonInRange;
  }

  public void setInRange(boolean isInRange) {
    autonInRange = isInRange;
  }
}
