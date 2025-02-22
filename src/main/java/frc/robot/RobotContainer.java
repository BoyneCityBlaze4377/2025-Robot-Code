package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.AutoAimConstants.Position;
import frc.robot.subsystems.*;
import frc.robot.commands.AllToSetPosition;
import frc.robot.commands.ClimberCommands.*;
import frc.robot.commands.DriveCommands.*;
import frc.robot.commands.ElevatorCommands.*;
import frc.robot.commands.PieceAffectorsCommands.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final Joystick m_driverStick = new Joystick(0);
  private final Joystick m_operatorStick = new Joystick(1);
  
  private final CoralAffector m_coralAffector = new CoralAffector();
  private final AlgaeAffector m_algaeAffector = new AlgaeAffector();
  private final Climber m_climber = new Climber();
  private final Elevator m_elevator = new Elevator();
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem(Constants.SensorConstants.limeLightName);
  private final DriveTrain m_driveTrain = new DriveTrain(m_elevator);

  private SendableChooser<Command> autonChooser = new SendableChooser<>();

  // private final AutoFactory autoFactory = new AutoFactory(m_driveTrain::getPose, m_driveTrain::resetOdometry, 
  //                                                         m_driveTrain::choreoDrive, true, m_driveTrain);

  private final Command TeleopDrive = new TeleopDrive(m_driverStick, m_driveTrain);
  private final Command LockPose = new LockPose(m_driveTrain); 
  private final Command SwitchBrake = new SwitchBrake(m_driveTrain);
  private final Command SwitchOrientation = new SwitchOrientation(m_driveTrain);
  private final Command QuickBrake = new QuickBrake(m_driveTrain);
  private final Command SlowMode = new SlowMode(m_driveTrain);

  // private final Command LLDrive = new LimeLightDrive(m_driveTrain, 1, 0, 0);

  private final Command AlgaeCollect = new AlgaeCollect(m_algaeAffector);
  private final Command AlgaeScore = new AlgaeScore(m_algaeAffector);

  private final Command CoralCollect = new CoralCollect(m_coralAffector);
  private final Command CoralScore = new CoralScore(m_coralAffector);
  private final Command CoralWrist = new CoralWristOverride(m_coralAffector, m_operatorStick);

  private final Command ElevatorUp = new ElevatorUp(m_elevator);
  private final Command ElevatorDown = new ElevatorDown(m_elevator);

  private final Command Climb = new Climb(m_climber, m_algaeAffector);
  private final Command UnClimb = new UnClimb(m_climber);

  private final Command ElevatorCommand = new frc.robot.commands.ElevatorCommands.ElevatorCommand(m_elevator, m_operatorStick);

  private final Command LockWrist = new LockCoralWrist(m_coralAffector);
  private final Command ZeroWrist = new ZeroCoralWrist(m_coralAffector);
  private final Command TestElevatorPos = new ElevatorToPosition(m_elevator, 100);
  private final Command TestCoralWristPos = new CoralWristToPos(m_coralAffector, 45);
  private final Command CoralWristOverride = new CoralWristOverride(m_coralAffector, m_driverStick);

  private final Command AllToFloor = new AllToSetPosition(m_elevator, m_coralAffector, Position.floor);
  private final Command AllToL1 = new AllToSetPosition(m_elevator, m_coralAffector, Position.L1);
  private final Command AllToL12Algae = new AllToSetPosition(m_elevator, m_coralAffector, Position.L12algae);
  private final Command AllToL2 = new AllToSetPosition(m_elevator, m_coralAffector, Position.L2);
  private final Command AllToL23Algae = new AllToSetPosition(m_elevator, m_coralAffector, Position.L23algae);
  private final Command AllToL3 = new AllToSetPosition(m_elevator, m_coralAffector, Position.L3);
  private final Command AllToL4 = new AllToSetPosition(m_elevator, m_coralAffector, Position.L4);
  private final Command AllToHP = new AllToSetPosition(m_elevator, m_coralAffector, Position.HP);

  // private final Command choreoCommand = choreoTEST();

  //private final SwerveSample trajectory;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() { 
    m_driveTrain.setDefaultCommand(TeleopDrive);
    configureButtonBindings();
    m_driveTrain.setGyroOffset(0);
    m_elevator.setDefaultCommand(ElevatorCommand);

    autonChooser.addOption("Test", null);
    IOConstants.ConfigTab.add(autonChooser);
  }

  public void setDriveTrainPoseEstimate() {
    m_driveTrain.setPoseEstimate(m_visionSubsystem.getEstimatedPose2d().get());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverStick, IOConstants.quickBrakeButtonID).whileTrue(QuickBrake);
    new JoystickButton(m_driverStick, IOConstants.slowModeButtonID).whileTrue(SlowMode);
    new JoystickButton(m_driverStick, IOConstants.lockPoseButtonID).whileTrue(LockPose);
    new JoystickButton(m_driverStick, IOConstants.switchOrientationButtonID).onTrue(SwitchOrientation);
    new JoystickButton(m_driverStick, IOConstants.switchBrakeButtonID).onTrue(SwitchBrake);

    new JoystickButton(m_driverStick, 11).onTrue(ZeroWrist);
    new JoystickButton(m_driverStick, 12).whileTrue(TestElevatorPos);
    new JoystickButton(m_driverStick, 9).whileTrue(TestCoralWristPos);

    // new JoystickButton(m_driverStick, 12).whileTrue(LLDrive);

    // new JoystickButton(m_operatorStick, 6).whileTrue(ElevatorUp);
    // new JoystickButton(m_operatorStick, 5).whileTrue(ElevatorDown);

    new JoystickButton(m_operatorStick, 4).whileTrue(CoralScore);
    new JoystickButton(m_operatorStick, 3).whileTrue(CoralCollect);
    // new JoystickButton(m_operatorStick, 2).whileTrue(AlgaeScore);
    new JoystickButton(m_operatorStick, 2).whileTrue(AlgaeCollect);

    new JoystickButton(m_operatorStick, 7).whileTrue(UnClimb);
    new JoystickButton(m_operatorStick, 8).whileTrue(Climb);

    new JoystickButton(m_operatorStick, 1).whileTrue(CoralWrist);
  }
  
  // public Command choreoTEST() {
  //   return Commands.sequence(autoFactory.trajectoryCmd("test 2 node"));
  // }

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