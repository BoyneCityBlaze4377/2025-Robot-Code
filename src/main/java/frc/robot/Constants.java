package frc.robot;

import java.util.HashMap;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.Lib.AdvancedPose2D;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public class IOConstants {
    public static final ShuffleboardTab TeleopTab = Shuffleboard.getTab("Teleop");
    public static final ShuffleboardTab AutonTab = Shuffleboard.getTab("Auton");
    public static final ShuffleboardTab DiagnosticTab = Shuffleboard.getTab("Diagnostic");
    public static final ShuffleboardTab ConfigTab = Shuffleboard.getTab("Configuration");

    /* CONTROLLER IDS */
    public static final int driverControllerID = 0;
    public static final int operatorController1ID = 2;
    public static final int operatorController2ID = 1;

    /* BUTTON IDS */
    /* Driver */
    public static final int quickBrakeButtonID = 6; //6
    public static final int slowModeButtonID = 2; //2
    public static final int switchBrakeButtonID = 5; //5
    public static final int switchOrientationButtonID = 3; //3
    public static final int lockPoseButtonID = 1; //4
    public static final int autoAlignButtonID = 0; //1

    /* Operator */
    // Positions
    public static final int floorPosButtonID = 1;
    public static final int L1PosButtonID = 2;
    public static final int L2AlgaePosButtonID = 5;
    public static final int L2PosButtonID = 4;
    public static final int L3AlgaePosButtonID = 7;
    public static final int L3PosButtonID = 6;
    public static final int L4PosButtonID = 8;
    public static final int HPPosButtonID = 3;

    public static final int elevatorOverrideButtonID = 12;

    // Affectors
    public static final int coralCollectButtonID = 1;
    public static final int coralScoreButtonID = 2;
    public static final int algaeCollectButtonID = 3;
    public static final int algaeScoreButtonID = 4;

    //Climber
    public static final int unClimbButtonID = 8;
    public static final int climbButtonID = 7;

    public static final int wristOverrideButtonID = 12;
  }

  public class ElevatorConstants {
    public static final int elevatorMotorID = 9;
    public static final int elevatorTwoID = 10;

    public static final double startingHeight = 0;

    public static final double conversionFactor = 1;

    public static final double lowerLimit = 5;
    public static final double upperLimit = 215;

    public static final double kP = .1;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kTolerance = 1;

    public static final double upSpeed = .4;
    public static final double downSpeed = -.4;
    public static final double overrideSpeed = .5;
    public static final double lockSpeed = .025;

    public static final double maxUpSpeed = .99;
    public static final double maxDownSpeed = -.93;
    public static final double correctionSpeed = .2;

    public static final double defaultPos = lowerLimit;
    public static final double floorPos = lowerLimit;
    public static final double L1Pos = 24;
    public static final double L2AlgaePos = 111;
    public static final double L2Pos = 85;
    public static final double L3AlgaePos = 177;
    public static final double L3Pos = 208;
    public static final double L4Pos = 208;
    public static final double HPPos = 120;
  }

  public class AffectorConstants {
    /** CORAL */
    public static final int coralAffectorID = 11;
    public static final int coralWristID = 12;

    public static final double startingAngle = 2;

    public static final double coralWristConversionFactor = (360 / 64);
    public static final double wristScoringThreshold = 90;

    public static final double coralWristDefaultPos = 0;
    public static final double coralWristL1 = 22;
    public static final double coralWristL3 = 21;
    public static final double coralWristL4 = 104;
    public static final double coralWristHP = 30;

    public static final double wristOverrideSpeed = .3;

    public static final double coralWristKP = .02; //.05
    public static final double coralWristKI = 0; //0
    public static final double coralWristKD = 0; //0
    public static final double coralWristKTolerance = 1;

    public static final double coralAffectorSpeed = .2;

    public static final double maxCoralWristUpSpeed = .25;
    public static final double maxCoralWristDownSpeed = -.05;

    /** ALGAE */
    public static final int algaeCollectorOneID = 13;
    public static final int algaeCollectorTwoID = 14;

    public static final double algaeCollectorSpeed = .6;
  }

  public static final class SwerveConstants {
    public static final double angleGearRatio = (150/7);
    public static final double voltageComp = 12.0;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 4.5; // meters per second  //4.5
    public static final double maxAngularVelocity =  11.5;

    /* Angle Motor PID Values */
    public static final double angleKP = 0.01; //.0005
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0;
    public static final double angleKFF = 0.0;
    public static final double kMaxOutput = 0.95;
    public static final double kTolerance = 1;

    public static final IdleMode angleNeutralMode = IdleMode.kBrake;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 20;

    public static final double angleConversionFactor = 360.0 / angleGearRatio;
  }

  public static final class DriveConstants {
    // Distance between centers of right and left wheels on robot in meters
    public static final double trackWidth = 0.31623;
    
    // Distance between front and back wheels on robot in meters
    public static final double wheelBase = 0.31623;
    
    public static final SwerveDriveKinematics driveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2, trackWidth / 2),
            new Translation2d(wheelBase / 2, -trackWidth / 2),
            new Translation2d(-wheelBase / 2, trackWidth / 2),
            new Translation2d(-wheelBase / 2, -trackWidth / 2));

    public static final boolean gyroReversed = true;

    public static final double speedScaler = 1;
    public static final double maxDriveSpeed = .96;
    public static final double minDriveSpeed = .35;
    public static final double maxRotSpeed = 1;
    public static final double minRotSpeed = .3;

    public static final double elevatorHeightFactorTranslation = (maxDriveSpeed - minDriveSpeed) / 
                                                                 (ElevatorConstants.upperLimit - ElevatorConstants.lowerLimit);
    public static final double elevatorHeightFactorRotation = (maxRotSpeed - minRotSpeed) / 
                                                              (ElevatorConstants.upperLimit - ElevatorConstants.lowerLimit);

    public static final double maxSpeedMetersPerSecond = 4; //4.5 true max
    public static final double maxAccelerationMetersPerSecondSquared = 1;
    public static final double maxRotationSpeedRadiansPerSecond = Math.PI * 2;

    public static final double xyDeadband = .1;
    public static final double zDeadband = .4;

    public static final double ksVolts = 5;
    public static final double kvVoltSecondsPerMeter = 4;
    public static final double kaVoltSecondsSquaredPerMeter = 1;

    public static final double startingHeading = 0;
  }

  public static final class ModuleConstants {
    public static final double moduleTurningController = .5;
    public static final double moduleDriveController = .75;
    public static final double moduleDriveSlewRate = 2;

    public static final int frontLeftDriveMotorPort = 1;
    public static final int frontRightDriveMotorPort = 3;
    public static final int backLeftDriveMotorPort = 7;
    public static final int backRightDriveMotorPort = 5;

    public static final int frontLeftTurningMotorPort = 2;
    public static final int frontRightTurningMotorPort = 4;
    public static final int backLeftTurningMotorPort = 8;
    public static final int backRightTurningMotorPort = 6;
    
    public static final int frontLeftTurningEncoderPort = 0;
    public static final int frontRightTurningEncoderPort = 1;
    public static final int backLeftTurningEncoderPort = 3;
    public static final int backRightTurningEncoderPort = 2;

    public static final boolean frontLeftTurningMotorReversed = false;    
    public static final boolean frontRightTurningMotorReversed = false;
    public static final boolean backLeftTurningMotorReversed = false;
    public static final boolean backRightTurningMotorReversed = false;

    public static final boolean frontLeftDriveMotorReversed = false;    
    public static final boolean frontRightDriveMotorReversed = false;
    public static final boolean backLeftDriveMotorReversed = false;
    public static final boolean backRightDriveMotorReversed = false;
    
    public static final boolean frontLeftAbsReversed = false;    
    public static final boolean frontRightAbsReversed = false;
    public static final boolean backLeftAbsReversed = false;
    public static final boolean backRightAbsReversed = false;

    public static final double frontLeftAnalogEncoderOffset = 4.26;  
    public static final double frontRightAnalogEncoderOffset = 76.44;
    public static final double backLeftAnalogEncoderOffset = 162.33;
    public static final double backRightAnalogEncoderOffset = 61.92;

    public static final double maxModuleAngularSpeedDegreesPerSecond = 360;
    public static final double maxModuleAngularAccelerationDegreesPerSecondSquared = 360;

    public static final double encoderCPR = 1;
    public static final double wheelDiameterMeters = Units.inchesToMeters(4);
    public static final double driveGearRatio = 1 / 6.75;
    public static final double driveEncoderDistancePerPulse = 
      // Assumes the encoders are directly mounted on the wheel shafts
      (wheelDiameterMeters * Math.PI) / (double) encoderCPR * driveGearRatio;
  }

  public class FieldConstants {
    public static final double fieldLength = 17.548;
    public static final double fieldWidth = 8.052;

    // public static AdvancedPose2D INIT_POSE_BLUE = new AdvancedPose2D(2, 4, Rotation2d.fromDegrees(0));
    public static AdvancedPose2D blueReefCenterPos = new AdvancedPose2D(4.48945, fieldWidth / 2, Rotation2d.fromDegrees(0));

    public static final AdvancedPose2D blueLeftStartAlgae = new AdvancedPose2D(new Translation2d(1.220, 5.848), new Rotation2d());
    public static final AdvancedPose2D blueCenterStartAlgae = new AdvancedPose2D(new Translation2d(1.220, 4.026), new Rotation2d());
    public static final AdvancedPose2D blueRightStartAlgae = new AdvancedPose2D(new Translation2d(1.220, 2.189), new Rotation2d());

    public static final AdvancedPose2D blueProcessor = new AdvancedPose2D(new Translation2d(6.348, .459), new Rotation2d(-Math.PI/2));
    public static final AdvancedPose2D redprocessor = blueProcessor.flipBoth();
  }

  public class AutoAimConstants{
    public static enum Position {floor, L1, L2algae, L2, L3algae, L3, L4, HP};
    public static enum ReefStation {front, frontRight, backRight, back, backLeft, frontLeft};
    public static enum ReefWaypoint {frontLeft, frontRight, right, backRight, backLeft, left, blank};
    public static enum Alignment {left, center, right};

    public static final HashMap<Position, double[]> positionValues = new HashMap<Position, double[]> () {{
      put(Position.floor, new double[] {ElevatorConstants.floorPos, AffectorConstants.coralWristDefaultPos});
      put(Position.L1, new double[] {ElevatorConstants.L1Pos, AffectorConstants.coralWristL1});
      put(Position.L2algae, new double[] {ElevatorConstants.L2AlgaePos, AffectorConstants.coralWristDefaultPos});
      put(Position.L2, new double[] {ElevatorConstants.L2Pos, AffectorConstants.coralWristL3});
      put(Position.L3algae, new double[] {ElevatorConstants.L3AlgaePos, AffectorConstants.coralWristDefaultPos});
      put(Position.L3, new double[] {ElevatorConstants.L3Pos, AffectorConstants.coralWristL3});
      put(Position.L4, new double[] {ElevatorConstants.L4Pos, AffectorConstants.coralWristL4});
      put(Position.HP, new double[] {ElevatorConstants.HPPos, AffectorConstants.coralWristHP});
    }};
            
    public static final double centerOfReefToRobotDistance = Units.inchesToMeters(32.75) + DriveConstants.trackWidth / 2 + 0.025;
    public static final double centerOfReefToWaypointDistance = Units.inchesToMeters(centerOfReefToRobotDistance) * 2 + 
                                                                 DriveConstants.trackWidth * Math.sqrt(2) + 
                                                                 Units.inchesToMeters(2);

    public static final double coralAffectorOffsetFromRobotCenter = Units.inchesToMeters(2);
    public static final double rightCoralReefOffset = Units.inchesToMeters(6.47) + coralAffectorOffsetFromRobotCenter;
    public static final double leftCoralReefOffset = coralAffectorOffsetFromRobotCenter - Units.inchesToMeters(6.47);

    public static final double LLDefaultOffsetDegrees = 2.3;
    public static final double LCToBumperEdgeOffsetMeters = Units.inchesToMeters(4.85);
    public static final double inRangeThreshold = .5; //Meters

    public static final double coralStationToRobotDistance = DriveConstants.trackWidth / 2 + .15;
    public static final double coralStationSideOffsetDistance = Units.inchesToMeters(76 / 4);

    public static final HashMap<Alignment, Double> offsetFromAlignment = new HashMap<Alignment, Double> () {{
      put(Alignment.left, leftCoralReefOffset);
      put(Alignment.center, 0.0);
      put(Alignment.right, rightCoralReefOffset);
    }};

    public static final double[] reefStationAngles = {0, 60, 120, 180, -180, -120, -60};
    public static final double[] reefWaypointAngles = {30, 90, 150, -150, -90, -30};
    public static final ReefWaypoint[] waypointArray = {ReefWaypoint.frontLeft, ReefWaypoint.frontRight, ReefWaypoint.right,
                                                        ReefWaypoint.backRight, ReefWaypoint.backLeft, ReefWaypoint.left};

    public static final HashMap<ReefStation, AdvancedPose2D> blueReef = new HashMap<ReefStation, AdvancedPose2D> () {{
      put(ReefStation.front, FieldConstants.blueReefCenterPos.withVector(Rotation2d.fromDegrees(0), new Translation2d(-centerOfReefToRobotDistance, 0), Rotation2d.fromDegrees(0)));
      put(ReefStation.frontRight, FieldConstants.blueReefCenterPos.withVector(Rotation2d.fromDegrees(-120), new Translation2d(centerOfReefToRobotDistance, 0), Rotation2d.fromDegrees(60)));
      put(ReefStation.backRight, FieldConstants.blueReefCenterPos.withVector(Rotation2d.fromDegrees(-60), new Translation2d(centerOfReefToRobotDistance, 0), Rotation2d.fromDegrees(120)));
      put(ReefStation.back, FieldConstants.blueReefCenterPos.withVector(Rotation2d.fromDegrees(0), new Translation2d(centerOfReefToRobotDistance, 0), Rotation2d.fromDegrees(180)));
      put(ReefStation.backLeft, FieldConstants.blueReefCenterPos.withVector(Rotation2d.fromDegrees(60), new Translation2d(centerOfReefToRobotDistance, 0), Rotation2d.fromDegrees(-120)));
      put(ReefStation.frontLeft, FieldConstants.blueReefCenterPos.withVector(Rotation2d.fromDegrees(120), new Translation2d(centerOfReefToRobotDistance, 0), Rotation2d.fromDegrees(-60)));
    }};

    public static final HashMap<ReefStation, AdvancedPose2D> redReef = new HashMap<ReefStation, AdvancedPose2D> () {{
      put(ReefStation.front, FieldConstants.blueReefCenterPos.horizontallyFlip().withVector(Rotation2d.fromDegrees(0), new Translation2d(-centerOfReefToRobotDistance, 0), Rotation2d.fromDegrees(0)));
      put(ReefStation.frontRight, FieldConstants.blueReefCenterPos.horizontallyFlip().withVector(Rotation2d.fromDegrees(-120), new Translation2d(centerOfReefToRobotDistance, 0), Rotation2d.fromDegrees(60)));
      put(ReefStation.backRight, FieldConstants.blueReefCenterPos.horizontallyFlip().withVector(Rotation2d.fromDegrees(-60), new Translation2d(centerOfReefToRobotDistance, 0), Rotation2d.fromDegrees(120)));
      put(ReefStation.back, FieldConstants.blueReefCenterPos.horizontallyFlip().withVector(Rotation2d.fromDegrees(0), new Translation2d(centerOfReefToRobotDistance, 0), Rotation2d.fromDegrees(180)));
      put(ReefStation.backLeft, FieldConstants.blueReefCenterPos.horizontallyFlip().withVector(Rotation2d.fromDegrees(60), new Translation2d(centerOfReefToRobotDistance, 0), Rotation2d.fromDegrees(-120)));
      put(ReefStation.backRight, FieldConstants.blueReefCenterPos.horizontallyFlip().withVector(Rotation2d.fromDegrees(120), new Translation2d(centerOfReefToRobotDistance, 0), Rotation2d.fromDegrees(-60)));
    }};

    public static final HashMap<ReefWaypoint, AdvancedPose2D> blueReefWaypoints = new HashMap<ReefWaypoint, AdvancedPose2D> () {{
      put(ReefWaypoint.frontLeft, FieldConstants.blueReefCenterPos.withVector(Rotation2d.fromDegrees(150), new Translation2d(centerOfReefToWaypointDistance, 0), Rotation2d.fromDegrees(0)));
      put(ReefWaypoint.frontRight, FieldConstants.blueReefCenterPos.withVector(Rotation2d.fromDegrees(-150), new Translation2d(centerOfReefToWaypointDistance, 0), Rotation2d.fromDegrees(0)));
      put(ReefWaypoint.right, FieldConstants.blueReefCenterPos.withVector(Rotation2d.fromDegrees(-90), new Translation2d(centerOfReefToWaypointDistance, 0), Rotation2d.fromDegrees(0)));
      put(ReefWaypoint.backRight, FieldConstants.blueReefCenterPos.withVector(Rotation2d.fromDegrees(-30), new Translation2d(centerOfReefToWaypointDistance, 0), Rotation2d.fromDegrees(0)));
      put(ReefWaypoint.backLeft, FieldConstants.blueReefCenterPos.withVector(Rotation2d.fromDegrees(30), new Translation2d(centerOfReefToWaypointDistance, 0), Rotation2d.fromDegrees(0)));
      put(ReefWaypoint.left, FieldConstants.blueReefCenterPos.withVector(Rotation2d.fromDegrees(90), new Translation2d(centerOfReefToWaypointDistance, 0), Rotation2d.fromDegrees(0)));
      put(ReefWaypoint.blank, new AdvancedPose2D(0, 0, new Rotation2d()));
    }};

    public static final HashMap<ReefWaypoint, AdvancedPose2D> redReefWaypoints = new HashMap<ReefWaypoint, AdvancedPose2D> () {{
      put(ReefWaypoint.frontLeft, FieldConstants.blueReefCenterPos.horizontallyFlip().withVector(Rotation2d.fromDegrees(150), new Translation2d(centerOfReefToWaypointDistance, 0), Rotation2d.fromDegrees(0)));
      put(ReefWaypoint.frontRight, FieldConstants.blueReefCenterPos.horizontallyFlip().withVector(Rotation2d.fromDegrees(-150), new Translation2d(centerOfReefToWaypointDistance, 0), Rotation2d.fromDegrees(0)));
      put(ReefWaypoint.right, FieldConstants.blueReefCenterPos.horizontallyFlip().withVector(Rotation2d.fromDegrees(-90), new Translation2d(centerOfReefToWaypointDistance, 0), Rotation2d.fromDegrees(0)));
      put(ReefWaypoint.backRight, FieldConstants.blueReefCenterPos.horizontallyFlip().withVector(Rotation2d.fromDegrees(-30), new Translation2d(centerOfReefToWaypointDistance, 0), Rotation2d.fromDegrees(0)));
      put(ReefWaypoint.backLeft, FieldConstants.blueReefCenterPos.horizontallyFlip().withVector(Rotation2d.fromDegrees(30), new Translation2d(centerOfReefToWaypointDistance, 0), Rotation2d.fromDegrees(0)));
      put(ReefWaypoint.left, FieldConstants.blueReefCenterPos.horizontallyFlip().withVector(Rotation2d.fromDegrees(90), new Translation2d(centerOfReefToWaypointDistance, 0), Rotation2d.fromDegrees(0)));
      put(ReefWaypoint.blank, new AdvancedPose2D(FieldConstants.fieldLength, 0, new Rotation2d()));
    }};

    public static final HashMap<Double, ReefStation> reefStationFromAngle = new HashMap<Double, ReefStation> () {{
      put(0.0, ReefStation.front);
      put(60.0, ReefStation.frontRight);
      put(120.0, ReefStation.backRight);
      put(180.0, ReefStation.back);
      put(-180.0, ReefStation.back);
      put(-120.0, ReefStation.backLeft);
      put(-60.0, ReefStation.frontLeft);
    }};

    public static final HashMap<ReefStation, Double> angleFromReefStation = new HashMap<ReefStation, Double> () {{
      put(ReefStation.front, 0.0);
      put(ReefStation.frontRight, 60.0);
      put(ReefStation.backRight, 120.0);
      put(ReefStation.back, 180.0);
      put(ReefStation.backLeft, -120.0);
      put(ReefStation.frontLeft, -60.0);
    }};

    public static final HashMap<Double, AdvancedPose2D> reefStationPoseFromAprilTagID = new HashMap<Double, AdvancedPose2D> () {{
      put(18., blueReef.get(ReefStation.front));
      put(17., blueReef.get(ReefStation.frontRight));
      put(22., blueReef.get(ReefStation.backRight));
      put(21., blueReef.get(ReefStation.back));
      put(20., blueReef.get(ReefStation.backLeft));
      put(19., blueReef.get(ReefStation.frontLeft));

      put(7., redReef.get(ReefStation.front));
      put(8., redReef.get(ReefStation.frontRight));
      put(9., redReef.get(ReefStation.backRight));
      put(10., redReef.get(ReefStation.back));
      put(11., redReef.get(ReefStation.backLeft));
      put(6., redReef.get(ReefStation.frontLeft));
    }};

    public static final HashMap<Double, ReefStation> reefStationFromAprilTagID = new HashMap<Double, ReefStation> () {{
      put(18., ReefStation.front);
      put(17., ReefStation.frontRight);
      put(22., ReefStation.backRight);
      put(21., ReefStation.back);
      put(20., ReefStation.backLeft);
      put(19., ReefStation.frontLeft);

      put(7., ReefStation.front);
      put(8., ReefStation.frontRight);
      put(9., ReefStation.backRight);
      put(10., ReefStation.back);
      put(11., ReefStation.backLeft);
      put(6., ReefStation.frontLeft);

      put(0., ReefStation.front);
    }};

    public static final HashMap<ReefStation, Double> blueReefIDsFromStation = new HashMap<ReefStation, Double>() {{
      put(ReefStation.front, 18.);
      put(ReefStation.frontRight, 17.);
      put(ReefStation.backRight, 22.);
      put(ReefStation.back, 21.);
      put(ReefStation.backLeft, 20.);
      put(ReefStation.frontLeft, 19.);
    }};

    public static final HashMap<ReefStation, Double> redReefIDsFromStation = new HashMap<ReefStation, Double>() {{
      put(ReefStation.front, 7.);
      put(ReefStation.frontRight, 8.);
      put(ReefStation.backRight, 9.);
      put(ReefStation.back, 10.);
      put(ReefStation.backLeft, 11.);
      put(ReefStation.frontLeft, 6.);
    }};

    public static final double blueReefWaypointBoundMinX = blueReefWaypoints.get(ReefWaypoint.frontRight).getX();
    public static final double blueReefWaypointBoundMaxX = blueReefWaypoints.get(ReefWaypoint.backRight).getX();
    public static final double redReefWaypointBoundMinX = redReefWaypoints.get(ReefWaypoint.backRight).getX();
    public static final double redReefWaypointBoundMaxX = redReefWaypoints.get(ReefWaypoint.frontRight).getX();
    public static final double reefWaypointBoundMinY = blueReefWaypoints.get(ReefWaypoint.right).getY();
    public static final double reefWaypointBoundMaxY = blueReefWaypoints.get(ReefWaypoint.left).getY();

    public static final double reefWaypointStaticDisMinY = blueReefWaypoints.get(ReefWaypoint.frontRight).getY();
    public static final double reefWaypointStaticDisMaxY = blueReefWaypoints.get(ReefWaypoint.backRight).getY();

    public static final double inReefCalculationInterval = .005;

    public static final AdvancedPose2D blueLeftCoralStationPos = new AdvancedPose2D(new Translation2d(0.836168, 0.6334625), null).withVector(Rotation2d.fromDegrees(54), new Translation2d(coralStationToRobotDistance, 0), Rotation2d.fromDegrees(-126));
    public static final AdvancedPose2D blueRightCoralStationPos = new AdvancedPose2D(new Translation2d(0.836168, 7.4185375), null).withVector(Rotation2d.fromDegrees(-54), new Translation2d(coralStationToRobotDistance, 0), Rotation2d.fromDegrees(126));
    public static final AdvancedPose2D redLeftCoralStationPos = blueLeftCoralStationPos.flipBoth();
    public static final AdvancedPose2D redRightCoralStationPos = blueRightCoralStationPos.flipBoth();

    public static final double horizkP = .05;
    public static final double horizkI = 0;
    public static final double horizkD = .002;
    public static final double horizkTolerance = 1.6;

    public static final double diskP = .004;
    public static final double diskI = .0008;
    public static final double diskD = 0;
    public static final double diskTolerance = 80;

    public static final double turnkP = 4.5; //4.5
    public static final double turnkI = .355; //.355
    public static final double turnkD = 0; //0
    public static final double turnkTolerance = .03;

    public static final double transkP = 1.5; //1.5
    public static final double transkI = .013; //.013
    public static final double transkD = 0; //0
    public static final double transkTolerance = .1;
  }

  public class ClimberConstants {
    public static final int climberControllerID = 15;
    public static final double climberspeed = .99;
  }

  public class AutonConstants {
    public static final double coralCollectTimeout = 3;
    public static final double coralScoreTime = .5;
    public static final double algaeCollectTimeout = 5;
    public static final double alageScoreTime = .5;

    public static final AdvancedPose2D initialPoseBlue = new AdvancedPose2D(new Pose2d(2, 3, new Rotation2d()));
    public static final AdvancedPose2D initialPoseRed = initialPoseBlue.flipBoth();
  }

  public class SensorConstants {
    /**LIMELIGHT */
    public static final String limeLightName = "limelight";

    /** LASERCAN */
    public static final int laserCANID = 17;

    /** BEAM BREAKS */
    public static final int coralBreakID = 0;
    public static final int algaeBreakID = 1;
  }
}