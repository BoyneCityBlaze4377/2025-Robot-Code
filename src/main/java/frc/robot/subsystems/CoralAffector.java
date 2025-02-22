package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AffectorConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.SensorConstants;

public class CoralAffector extends SubsystemBase {
  private final SparkMax coralAffector, coralWrist;
  private final SparkMaxConfig coralAffectorConfig, coralWristConfig;
  private final RelativeEncoder wristEncoder;
  private final DigitalInput coralDetector;

  /** Creates a new PieceAffector. */
  public CoralAffector() {
    coralAffector = new SparkMax(AffectorConstants.coralAffectorID, MotorType.kBrushless);
    coralWrist = new SparkMax(AffectorConstants.coralWristID, MotorType.kBrushless);

    coralAffectorConfig = new SparkMaxConfig();
    coralWristConfig = new SparkMaxConfig();

    wristEncoder = coralWrist.getEncoder();

    configMotorControllerDefaults();

    coralDetector = new DigitalInput(SensorConstants.coralBreakID);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("WristEncoderDeg", getWristDegrees());
    SmartDashboard.putBoolean("HasCoral", hasCoral());
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

  public void moveWrist(double speed) {
    coralWrist.set(speed);
  }

  public void collect() {
    coralAffector.set(AffectorConstants.coralAffectorSpeed);
  }

  public void eject() {
    if (getWristDegrees() > AffectorConstants.wristScoringThreshold) {
      coralAffector.set(AffectorConstants.coralAffectorSpeed);
    } else {
      coralAffector.set(-AffectorConstants.coralAffectorSpeed);
    }
  }

  public void setAffector(double speed) {
    coralAffector.set(speed);
  }

  public void stopAffector() {
    coralAffector.set(0);
  }

  public void stopWrist() {
    coralWrist.set(0);
  }

  public double getWristDegrees() {
    return wristEncoder.getPosition() * AffectorConstants.coralWristConversionFactor;
  }

  public void lockWrist() {
    coralWrist.set(.06 - .000006 * Math.pow(getWristDegrees() - 90, 2));
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
}
