package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import frc.Lib.AdvancedPose2D;

import frc.robot.Constants.AutoAimConstants;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.SensorConstants;
import frc.robot.Constants.AutoAimConstants.Alignment;
import frc.robot.Constants.AutoAimConstants.Position;
import frc.robot.Constants.AutoAimConstants.ReefStation;
import frc.robot.subsystems.*;
import frc.robot.commands.AllToSetPosition;
import frc.robot.commands.ClimberCommands.*;
import frc.robot.commands.DriveCommands.*;
import frc.robot.commands.ElevatorCommands.*;
import frc.robot.commands.PieceAffectorsCommands.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final Alliance alliance = DriverStation.getAlliance().get();
  private final HashMap<ReefStation, AdvancedPose2D> reef = alliance == Alliance.Blue ? AutoAimConstants.blueReef : AutoAimConstants.redReef;

  private final Joystick m_driverStick = new Joystick(IOConstants.driverControllerID); //Driving
  private final Joystick m_operatorStick1 = new Joystick(IOConstants.operatorController1ID); //Set positions and elevatorOverride
  private final Joystick m_operatorStick2 = new Joystick(IOConstants.operatorController2ID); //Affectors, climber, alignments, and wristOverride
  
  private final CoralAffector m_coralAffector = new CoralAffector();
  private final AlgaeAffector m_algaeAffector = new AlgaeAffector();
  private final Climber m_climber = new Climber();
  private final Elevator m_elevator = new Elevator();
  private final AutoAimSubsystem m_autoAimSubsystem = new AutoAimSubsystem(Constants.SensorConstants.limeLightName);
  private final DriveTrain m_driveTrain = new DriveTrain(m_elevator, m_autoAimSubsystem, SensorConstants.limeLightName, 
                                                         alliance, m_driverStick);

  private SendableChooser<Command> autonChooser = new SendableChooser<Command>();
  private SendableChooser<AdvancedPose2D> initialPoseSelector = new SendableChooser<AdvancedPose2D>();

  /* COMMANDS */
  // DriveTrain
  //Main Commands
  private final Command TeleopDrive = new TeleopDrive(m_driverStick, m_driveTrain);
  private final Command LockPose = new LockPose(m_driveTrain); 
  private final Command SwitchBrake = new SwitchBrake(m_driveTrain);
  private final Command SwitchOrientation = new SwitchOrientation(m_driveTrain);
  private final Command QuickBrake = new QuickBrake(m_driveTrain);
  private final Command SlowMode = new SlowMode(m_driveTrain);
  private final Command StraightDrive = new StraightDrive(m_driveTrain, m_driverStick);
  private final Command AutoAimDrive = new AutoAimDrive(m_driveTrain, m_autoAimSubsystem);
  private final Command TEMPAUTODRIVE = new TEMPAUTODRIVE(m_driveTrain);

  //Poses
  private final Command SelectAlignmentLeft = new SelectDesiredAlignment(m_autoAimSubsystem, Alignment.left);
  private final Command SelectAlignmentCenter = new SelectDesiredAlignment(m_autoAimSubsystem, Alignment.center);
  private final Command SelectAlignmentRight = new SelectDesiredAlignment(m_autoAimSubsystem, Alignment.right);

  private final Command SelectReefStationFront = new SelectDesiredPose(m_autoAimSubsystem, reef.get(ReefStation.front));
  private final Command SelectReefStationFrontRight = new SelectDesiredPose(m_autoAimSubsystem, reef.get(ReefStation.frontRight));
  private final Command SelectReefStationBackRight = new SelectDesiredPose(m_autoAimSubsystem, reef.get(ReefStation.backRight));
  private final Command SelectReefStationBack = new SelectDesiredPose(m_autoAimSubsystem, reef.get(ReefStation.back));
  private final Command SelectReefStationBackLeft = new SelectDesiredPose(m_autoAimSubsystem, reef.get(ReefStation.backLeft));
  private final Command SelectReefStationFrontLeft = new SelectDesiredPose(m_autoAimSubsystem, reef.get(ReefStation.frontLeft));
  private final Command SelectProcessorPose = new SelectDesiredPose(m_autoAimSubsystem, alliance == Alliance.Blue ?
                                                                      FieldConstants.blueProcessor.withRobotRelativeTransformation(
                                                                        new Translation2d(0, AutoAimConstants.algaePosBackset)) :
                                                                      FieldConstants.redprocessor.withRobotRelativeTransformation(
                                                                        new Translation2d(0, AutoAimConstants.algaePosBackset)));

  //Positions
  private final Command AllToFloor = new AllToSetPosition(m_elevator, m_coralAffector, Position.floor);
  private final Command AllToL1 = new AllToSetPosition(m_elevator, m_coralAffector, Position.L1);
  private final Command AllToL2Algae = new AllToSetPosition(m_elevator, m_coralAffector, Position.L2algae);
  private final Command AllToL2 = new AllToSetPosition(m_elevator, m_coralAffector, Position.L2);
  private final Command AllToL3Algae = new AllToSetPosition(m_elevator, m_coralAffector, Position.L3algae);
  private final Command AllToL3 = new AllToSetPosition(m_elevator, m_coralAffector, Position.L3);
  private final Command AllToL4 = new AllToSetPosition(m_elevator, m_coralAffector, Position.L4);
  private final Command AllToHP = new AllToSetPosition(m_elevator, m_coralAffector, Position.HP);

  //Overrides
  private final Command ElevatorOverride = new ElevatorOverride(m_elevator, m_operatorStick1);
  private final Command WristOverride = new CoralWristOverride(m_coralAffector, m_operatorStick2);

  //Affectors
  private final Command CoralCollect = new CoralCollect(m_coralAffector);
  private final Command CoralScore = new CoralScore(m_coralAffector);
  private final Command AlgaeCollect = new AlgaeCollect(m_algaeAffector);
  private final Command AlgaeScore = new AlgaeScore(m_algaeAffector);

  //Climber
  private final Command Climb = new Climb(m_climber, m_algaeAffector);
  private final Command UnClimb = new UnClimb(m_climber);

  //Auton

  //Testing

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_driveTrain.setDefaultCommand(TeleopDrive);
    configureButtonBindings();
    m_driveTrain.setOdometry(new Pose2d());

    configAutonChooser();
    IOConstants.ConfigTab.add("Auton Chooser", autonChooser);

    configInitialPoseSelector();
    IOConstants.ConfigTab.add("Starting Position Chooser", initialPoseSelector);
  }

  public void setDriveTrainInitialPose() {
    m_driveTrain.setInitialPose(initialPoseSelector.getSelected());
  }

  public void setDriveTrainPoseEstimate() {
    //if (m_autoAimSubsystem.getEstimatedGlobalPose().isPresent()) m_driveTrain.setOdometry(m_autoAimSubsystem.getEstimatedPose2d().get());
  }

  public void setDriveOrientation(boolean fieldOriented) {
    m_driveTrain.setOrientation(fieldOriented);
  }

  public void configAutonChooser() {
    autonChooser.setDefaultOption("No Auton", null);
  }

  public void configInitialPoseSelector() {
    initialPoseSelector.setDefaultOption("Blue, Right", AutonConstants.initialPoseBlueRight);
    initialPoseSelector.addOption("Blue, Left", AutonConstants.initialPoseBlueLeft);
    initialPoseSelector.addOption("Red, Right", AutonConstants.initialPoseRedRight);
    initialPoseSelector.addOption("Red, Left", AutonConstants.initialPoseRedLeft);
    initialPoseSelector.addOption("Custom", AutonConstants.customInitialPose);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    /* DRIVER */
    //Main Commands
    new JoystickButton(m_driverStick, IOConstants.quickBrakeButtonID).whileTrue(QuickBrake);
    new JoystickButton(m_driverStick, IOConstants.slowModeButtonID).whileTrue(SlowMode);
    new JoystickButton(m_driverStick, IOConstants.lockPoseButtonID).whileTrue(LockPose);
    new JoystickButton(m_driverStick, IOConstants.switchOrientationButtonID).onTrue(SwitchOrientation);
    new JoystickButton(m_driverStick, IOConstants.switchBrakeButtonID).onTrue(SwitchBrake);
    new JoystickButton(m_driverStick, IOConstants.StraightDriveButtonID).whileTrue(StraightDrive);
    new JoystickButton(m_driverStick, IOConstants.autoDriveButtonID).whileTrue(TEMPAUTODRIVE);

    //Poses

    /* OPERATOR */
    //Set positions
    new JoystickButton(m_operatorStick1, IOConstants.floorPosButtonID).whileTrue(AllToFloor);
    new JoystickButton(m_operatorStick1, IOConstants.L1PosButtonID).whileTrue(AllToL1);
    new JoystickButton(m_operatorStick1, IOConstants.L2AlgaePosButtonID).whileTrue(AllToL2Algae);
    new JoystickButton(m_operatorStick1, IOConstants.L2PosButtonID).whileTrue(AllToL2);
    new JoystickButton(m_operatorStick1, IOConstants.L3AlgaePosButtonID).whileTrue(AllToL3Algae);
    new JoystickButton(m_operatorStick1, IOConstants.L3PosButtonID).whileTrue(AllToL3);
    new JoystickButton(m_operatorStick1, IOConstants.L4PosButtonID).whileTrue(AllToL4);
    new JoystickButton(m_operatorStick1, IOConstants.HPPosButtonID).whileTrue(AllToHP);

    //Overrides
    new JoystickButton(m_operatorStick1, IOConstants.elevatorOverrideButtonID).whileTrue(ElevatorOverride);
    new JoystickButton(m_operatorStick2, IOConstants.wristOverrideButtonID).whileTrue(WristOverride);

    //Affectors
    new JoystickButton(m_operatorStick2, IOConstants.coralCollectButtonID).whileTrue(CoralCollect);
    new JoystickButton(m_operatorStick2, IOConstants.coralScoreButtonID).whileTrue(CoralScore);
    new JoystickButton(m_operatorStick2, IOConstants.algaeCollectButtonID).whileTrue(AlgaeCollect);
    new JoystickButton(m_operatorStick2, IOConstants.algaeScoreButtonID).whileTrue(AlgaeScore);

    //Climber
    new JoystickButton(m_operatorStick2, IOConstants.unClimbButtonID).whileTrue(UnClimb);
    new JoystickButton(m_operatorStick2, IOConstants.climbButtonID).whileTrue(Climb);

    //Testing

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }
}