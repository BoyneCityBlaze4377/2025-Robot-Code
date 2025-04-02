package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  private final SparkMax climberMotor;
  private final SparkMaxConfig config;

  /** Creates a new Climber. */
  public Climber() {
    /** Motor */
    climberMotor = new SparkMax(ClimberConstants.climberControllerID, MotorType.kBrushless);
    config = new SparkMaxConfig();
    configure();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /** Configure the climber's motor controller */
  private void configure() {
    config.idleMode(IdleMode.kBrake);
    config.inverted(false);
    climberMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /** Pull the climber in */
  public void climb() {
    climberMotor.set(ClimberConstants.climberspeed);
  }

  /** Let the climber out */
  public void unClimb() {
    climberMotor.set(-ClimberConstants.climberspeed);
  }

  /** Stop the climber */
  public void stop() {
    climberMotor.set(0);
  }
}
