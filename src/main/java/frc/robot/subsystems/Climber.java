package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  private final SparkMax climberMotor;
  private final SparkMaxConfig config;
  private final RelativeEncoder climberEncoder;

  /** Creates a new Climber. */
  public Climber() {
    climberMotor = new SparkMax(ClimberConstants.climberControllerID, MotorType.kBrushless);
    config = new SparkMaxConfig();
    climberEncoder = climberMotor.getEncoder();

    configure();
    // zeroEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void configure() {
    config.idleMode(IdleMode.kBrake);
    config.inverted(false);
    climberMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void climb() {
    climberMotor.set(ClimberConstants.climberspeed);
  }

  public void unClimb() {
    climberMotor.set(-ClimberConstants.climberspeed);
  }

  public void stop() {
    climberMotor.set(0);
  }

  public double getEncoderPos() {
    return climberEncoder.getPosition() / 108;
  }

  public void zeroEncoder() {
    climberEncoder.setPosition(0);
  }
}
