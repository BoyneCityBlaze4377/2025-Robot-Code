// package frc.robot.commands.Auton;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.DriveConstants;
// import frc.robot.subsystems.DriveTrain;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class AutonLimeLightDrive extends Command {
//   private final DriveTrain m_driveTrain;
//   private double x, d, xSpeed, ySpeed, rot, tarOffsetDegrees;
//   private final double turnP, distanceP, xOffsetP;
//   private final double targetDistance, targetDegrees;
//   private double[] limelightArray;
//   private final PIDController angleController, xController, distanceController;

//   /** Creates a new LimeLightDrive. */
//   public AutonLimeLightDrive(DriveTrain driveTrain, double TargetDistance, double TargetDegrees, double targetOffset) {
//     m_driveTrain = driveTrain;
//     targetDistance = TargetDistance;
//     targetDegrees = TargetDegrees;
//     tarOffsetDegrees = Math.asin(targetOffset / targetDegrees);
//     angleController = new PIDController(.5, 0, 0);
//     xController = new PIDController(.25, 0, 0);
//     distanceController = new PIDController(.5, 0, 0);
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(m_driveTrain);

//     turnP = 0;
//     distanceP = .001;
//     xOffsetP = 0;
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     angleController.setSetpoint(targetDegrees);
//     distanceController.setSetpoint(targetDistance);
//     xController.setSetpoint(tarOffsetDegrees);

//     angleController.setTolerance(1);
//     distanceController.setTolerance(.1);
//     xController.setTolerance(1);
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     limelightArray = m_driveTrain.getLimelightValues();
//     x = limelightArray[0];
//     d = limelightArray[2] * DriveConstants.LLAreaToDistanceConversionFactor;

//     rot = (m_driveTrain.getHeading() - targetDegrees) * turnP;
//     xSpeed = d * distanceP;
//     ySpeed = (x - tarOffsetDegrees) * xOffsetP;

//     if (Math.abs(xSpeed) > .2) {
//       xSpeed = Math.copySign(.2, xSpeed);
//     }

//     if (Math.abs(ySpeed) > .2) {
//       ySpeed = Math.copySign(.2, ySpeed);
//     }

//     if (Math.abs(rot) > .3) {
//       rot = Math.copySign(.3, rot);
//     }

//     m_driveTrain.autonDrive(xSpeed, 0, 0);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     m_driveTrain.drive(0, 0, 0);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
