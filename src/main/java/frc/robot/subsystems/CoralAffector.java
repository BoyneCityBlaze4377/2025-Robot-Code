package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AffectorConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.SensorConstants;

public class CoralAffector extends SubsystemBase {
  private final SparkMax coralAffector, coralWrist;
  private final SparkMaxConfig coralAffectorConfig, coralWristConfig;
  private final RelativeEncoder wristEncoder;
  private final DigitalInput coralDetector;
  private final GenericEntry wristValSender, hasCoralSender, lockedSender;
  private boolean locked;
  private final double dA;
  private final PIDController wristController;
  private double wristSpeed;

  /** Creates a new PieceAffector. */
  public CoralAffector() {
    coralAffector = new SparkMax(AffectorConstants.coralAffectorID, MotorType.kBrushless);
    coralWrist = new SparkMax(AffectorConstants.coralWristID, MotorType.kBrushless);

    coralAffectorConfig = new SparkMaxConfig();
    coralWristConfig = new SparkMaxConfig();

    wristEncoder = coralWrist.getEncoder();

    wristController = new PIDController(AffectorConstants.coralWristKP, 
                                        AffectorConstants.coralWristKI, 
                                        AffectorConstants.coralWristKD);
    wristController.setTolerance(AffectorConstants.coralWristKTolerance);

    dA = AffectorConstants.startingAngle - wristEncoder.getPosition() * AffectorConstants.coralWristConversionFactor;

    configMotorControllerDefaults();

    coralDetector = new DigitalInput(SensorConstants.coralBreakID);

    wristValSender = IOConstants.TeleopTab.add("Wrist Encoder Degrees", getWristDegrees())
                                              .withWidget("Radial Gauge")
                                              .withProperties(Map.of("start_angle", 180, "end_angle", 0,
                                                                     "min_value", 0, "max_value", 180, 
                                                                     "number_of_labels", 4, "show_pointer", false))
                                              .getEntry();
    hasCoralSender = IOConstants.TeleopTab.add("HasCoral", hasCoral()).withWidget("Boolean Box").getEntry();
    lockedSender = IOConstants.DiagnosticTab.add("CoralWrist Locked", false)
                                            .withWidget("Boolean Box").getEntry();

    wristSpeed = 0;
  }
  
  @Override
  public void periodic() {
    wristValSender.setDouble(getWristDegrees());
    hasCoralSender.setBoolean(hasCoral());
    lockedSender.setBoolean(locked);

    coralWrist.set(wristSpeed);
  }

  private void configMotorControllerDefaults() {
    coralAffectorConfig.inverted(true);
    coralAffectorConfig.idleMode(IdleMode.kCoast);

    coralWristConfig.inverted(false);
    coralWristConfig.idleMode(IdleMode.kBrake);

    configMotorControllers();
  }

  private void configMotorControllers() {
    coralAffector.configure(coralAffectorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    coralWrist.configure(coralWristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void overrideWrist(double speed) {
    wristSpeed = speed;
    locked = false;
  }

  public void collect() {
    coralAffector.set(AffectorConstants.coralAffectorInSpeed);
  }

  public void eject() {
    if (getWristDegrees() > AffectorConstants.wristScoringThreshold) {
      coralAffector.set(AffectorConstants.coralAffectorOutSpeed);
    } else {
      coralAffector.set(-AffectorConstants.coralAffectorOutSpeed);
    }
  }

  public void setAffector(double speed) {
    coralAffector.set(speed);
  }

  public void stopAffector() {
    coralAffector.set(0);
  }

  public void stopWrist() {
    wristSpeed = 0;
    locked = false;
  }

  public double getWristDegrees() {
    return wristEncoder.getPosition() * AffectorConstants.coralWristConversionFactor + dA;
  }

  public void overrideLockWrist() {
    wristSpeed = (.068 - .0000052 * Math.pow(getWristDegrees() - 90, 2));
    locked = true;
  }

  public void PIDLockWrist() {
    wristSpeed = (.068 - .0000052 * Math.pow(wristController.getSetpoint() - 90, 2));
    locked = true;
  }

  public boolean getDetector() {
    return coralDetector.get();
  }

  public boolean hasCoral() {
    return !coralDetector.get();
  }

  public void zeroWristEncoder() {
    wristEncoder.setPosition(0);
  }

  public void setSetpoint(double setPoint) {
    wristController.setSetpoint(setPoint);
  }

  public void PIDMoveWrist() {
    wristSpeed = (MathUtil.clamp(wristController.calculate(getWristDegrees()), 
                  AffectorConstants.maxCoralWristDownSpeed, AffectorConstants.maxCoralWristUpSpeed));
    locked = false;
  }

  public boolean atSetpoint() {
    return wristController.atSetpoint();
  }
}
