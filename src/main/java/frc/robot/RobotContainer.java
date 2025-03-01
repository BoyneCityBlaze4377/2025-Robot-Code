package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.AutoAimConstants.Alignment;
import frc.robot.Constants.AutoAimConstants.Position;
import frc.robot.subsystems.*;
import frc.robot.commands.AllToSetPosition;
import frc.robot.commands.Auton.Functions.AutonAutoAlign;
import frc.robot.commands.Auton.Functions.AutonDrive;
import frc.robot.commands.Auton.Functions.FirstAutonDrive;
import frc.robot.commands.Auton.Sequences.*;
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

  private final Joystick m_driverStick = new Joystick(IOConstants.driverControllerID); //Driving
  private final Joystick m_operatorStick1 = new Joystick(IOConstants.operatorController1ID); //Set positions and elevatorOverride
  private final Joystick m_operatorStick2 = new Joystick(IOConstants.operatorController2ID); //Affectors, climber, and wristOverride
  
  private final CoralAffector m_coralAffector = new CoralAffector();
  private final AlgaeAffector m_algaeAffector = new AlgaeAffector();
  private final Climber m_climber = new Climber();
  private final Elevator m_elevator = new Elevator();
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem(Constants.SensorConstants.limeLightName);
  private final DriveTrain m_driveTrain = new DriveTrain(m_elevator);

  private SendableChooser<Command> autonChooser = new SendableChooser<>();

  private final Command TeleopDrive = new TeleopDrive(m_driverStick, m_driveTrain);
  private final Command LockPose = new LockPose(m_driveTrain); 
  private final Command SwitchBrake = new SwitchBrake(m_driveTrain);
  private final Command SwitchOrientation = new SwitchOrientation(m_driveTrain);
  private final Command QuickBrake = new QuickBrake(m_driveTrain);
  private final Command SlowMode = new SlowMode(m_driveTrain);
  private final Command AutoAlign = new AutoAlign(m_driveTrain, m_visionSubsystem, 0, 0);

  // private final Command ElevatorOverride = new ElevatorOverride(m_elevator, m_operatorStick);
  // private final Command CoralWristOverride = new CoralWristOverride(m_coralAffector, m_driverStick);

  private final Command AllToFloor = new AllToSetPosition(m_elevator, m_coralAffector, Position.floor);
  private final Command AllToL1 = new AllToSetPosition(m_elevator, m_coralAffector, Position.L1);
  private final Command AllToL2Algae = new AllToSetPosition(m_elevator, m_coralAffector, Position.L2algae);
  private final Command AllToL2 = new AllToSetPosition(m_elevator, m_coralAffector, Position.L2);
  private final Command AllToL3Algae = new AllToSetPosition(m_elevator, m_coralAffector, Position.L3algae);
  private final Command AllToL3 = new AllToSetPosition(m_elevator, m_coralAffector, Position.L3);
  private final Command AllToL4 = new AllToSetPosition(m_elevator, m_coralAffector, Position.L4);
  private final Command AllToHP = new AllToSetPosition(m_elevator, m_coralAffector, Position.HP);

  private final Command CoralCollect = new CoralCollect(m_coralAffector);
  private final Command CoralScore = new CoralScore(m_coralAffector);
  private final Command AlgaeCollect = new AlgaeCollect(m_algaeAffector);
  private final Command AlgaeScore = new AlgaeScore(m_algaeAffector);

  private final Command Climb = new Climb(m_climber, m_algaeAffector);
  private final Command UnClimb = new UnClimb(m_climber);

  private final Command ElevatorOverride = new ElevatorOverride(m_elevator, m_operatorStick1);
  private final Command WristOverride = new CoralWristOverride(m_coralAffector, m_operatorStick2);

  /* Auton */
  private final Command MainAutonEndCoral = new MainAutonEndAlgae(m_driveTrain, m_elevator, m_coralAffector, 
                                                                  m_algaeAffector, m_visionSubsystem, alliance);
  private final Command MainAutonEndAlgae = new MainAutonEndCoral(m_driveTrain, m_elevator, m_coralAffector, 
                                                                  m_visionSubsystem, alliance);
  private final Command DriveOffLine = new DriveOffLine(m_driveTrain);
  private final Command Score1CoralL4 = new Score1CoralL4(m_driveTrain, m_elevator, m_coralAffector, m_visionSubsystem, alliance);
  private final Command ScoreCoralAndProcessor = new Score1L4AndProcessor(m_driveTrain, m_elevator, m_coralAffector, 
                                                                          m_algaeAffector, m_visionSubsystem, alliance);

  private final Command testdrive = new FirstAutonDrive(m_driveTrain, 180, 4, Math.PI * 2, 
                                                        Units.inchesToMeters(104), 120);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() { 
    m_driveTrain.setDefaultCommand(TeleopDrive);
    configureButtonBindings();
    m_driveTrain.setGyroOffset(0);

    configAutonChooser();
    IOConstants.ConfigTab.add("Auton Chooser", autonChooser);
  }

  public void setDriveTrainPoseEstimate() {
    m_driveTrain.setPoseEstimate(m_visionSubsystem.getEstimatedPose2d().get());
  }

  public void configAutonChooser() {
    autonChooser.setDefaultOption("No Auton", null);
    autonChooser.addOption("Main Auton - Coral Ending", MainAutonEndCoral);
    autonChooser.addOption("Main Auton - Algae Ending", MainAutonEndAlgae);
    autonChooser.addOption("Drive Off Line", DriveOffLine);
    autonChooser.addOption("Score One on L4", Score1CoralL4);
    autonChooser.addOption("Score one Coral, Process Algae", ScoreCoralAndProcessor);
    autonChooser.addOption("TEST", testdrive);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    /* DRIVER */
    new JoystickButton(m_driverStick, IOConstants.quickBrakeButtonID).whileTrue(QuickBrake);
    new JoystickButton(m_driverStick, IOConstants.slowModeButtonID).whileTrue(SlowMode);
    new JoystickButton(m_driverStick, IOConstants.lockPoseButtonID).whileTrue(LockPose);
    new JoystickButton(m_driverStick, IOConstants.switchOrientationButtonID).onTrue(SwitchOrientation);
    new JoystickButton(m_driverStick, IOConstants.switchBrakeButtonID).onTrue(SwitchBrake);
    // new JoystickButton(m_driverStick, IOConstants.autoAlignButtonID).whileTrue(new AutoAlign(m_driveTrain, 
    //                                                                                          m_visionSubsystem, 
    //                                                                                          .01, 
    //                                                                                          m_driverStick.getPOV()));
    // new JoystickButton(m_driverStick, 12).whileTrue(testdrive);

    /* Operator */
    //Set positions
    new JoystickButton(m_operatorStick1, IOConstants.floorPosButtonID).whileTrue(AllToFloor);
    new JoystickButton(m_operatorStick1, IOConstants.L1PosButtonID).whileTrue(AllToL1);
    new JoystickButton(m_operatorStick1, IOConstants.L2AlgaePosButtonID).whileTrue(AllToL2Algae);
    new JoystickButton(m_operatorStick1, IOConstants.L2PosButtonID).whileTrue(AllToL2);
    new JoystickButton(m_operatorStick1, IOConstants.L3AlgaePosButtonID).whileTrue(AllToL3Algae);
    new JoystickButton(m_operatorStick1, IOConstants.L3PosButtonID).whileTrue(AllToL3);
    new JoystickButton(m_operatorStick1, IOConstants.L4PosButtonID).whileTrue(AllToL4);
    new JoystickButton(m_operatorStick1, IOConstants.HPPosButtonID).whileTrue(AllToHP);

    //Affectors
    new JoystickButton(m_operatorStick2, IOConstants.coralCollectButtonID).whileTrue(CoralCollect);
    new JoystickButton(m_operatorStick2, IOConstants.coralScoreButtonID).whileTrue(CoralScore);
    new JoystickButton(m_operatorStick2, IOConstants.algaeCollectButtonID).whileTrue(AlgaeCollect);
    new JoystickButton(m_operatorStick2, IOConstants.algaeScoreButtonID).whileTrue(AlgaeScore);

    //Climber
    new JoystickButton(m_operatorStick2, IOConstants.unClimbButtonID).whileTrue(UnClimb);
    new JoystickButton(m_operatorStick2, IOConstants.climbButtonID).whileTrue(Climb);

    //Overrides
    new JoystickButton(m_operatorStick1, IOConstants.elevatorOverrideButtonID).whileTrue(ElevatorOverride);
    new JoystickButton(m_operatorStick2, IOConstants.wristOverrideButtonID).whileTrue(WristOverride);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // var thetaController = new PIDController(AutoConstants.kPThetaController, 0, 0);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // m_robotDrive.resetOdometry(traj.getInitialPose());

    // Command swerveCommand = Choreo.choreoSwerveCommand(
    //     traj, // Choreo trajectory from above
    //     m_robotDrive::getPose, // A function that returns the current field-relative pose of the robot: your
    //                            // wheel or vision odometry
    //     new PIDController(Constants.AutoConstants.kPXController, 0.0, 0.0), // PIDController for field-relative X
    //                                                                                // translation (input: X error in meters,
    //                                                                                // output: m/s).
    //     new PIDController(Constants.AutoConstants.kPYController, 0.0, 0.0), // PIDController for field-relative Y
    //                                                                                // translation (input: Y error in meters,
    //                                                                                // output: m/s).
    //     thetaController, // PID constants to correct for rotation
    //                      // error
    //     (ChassisSpeeds speeds) -> m_robotDrive.drive( // needs to be robot-relative
    //         speeds.vxMetersPerSecond,
    //         speeds.vyMetersPerSecond,
    //         speeds.omegaRadiansPerSecond,
    //         false),
    //     true, // Whether or not to mirror the path based on alliance (this assumes the path is created for the blue alliance)
    //     m_driveTrain // The subsystem(s) to require, typically your drive subsystem only
    // );

    // return Commands.sequence(
    //     Commands.runOnce(() -> m_driveTrain.resetOdometry(traj.getInitialPose())),
    //     swerveCommand,
    //     m_robotDrive.run(() -> m_robotDrive.drive(0, 0, 0, false))
    // );
    return autonChooser.getSelected();
  }
}