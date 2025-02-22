package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  private final SparkMax elevatorMotor;
  private final SparkMaxConfig elevatorMotorConfig;
  private final RelativeEncoder elevatorEncoder;
  private double elevatorSpeed;

  /** Creates a new Elevator. */
  public Elevator() {
    elevatorMotor = new SparkMax(ElevatorConstants.elevatorMotorID, MotorType.kBrushless);

    elevatorMotorConfig = new SparkMaxConfig();

    elevatorEncoder = elevatorMotor.getEncoder();
    elevatorSpeed = 0;

    configMotorControllerDefaults();
  }

  @Override
  public void periodic() {
    if (atUpperLimit()) {
      elevatorMotor.set(-ElevatorConstants.correctionSpeed);
    } else if (atLowerLimit()) {
      elevatorMotor.set(ElevatorConstants.correctionSpeed);
    } else {
      elevatorMotor.set(elevatorSpeed);
    }
  }

  public void set(double speed) {
    if (!atLowerLimit() && !atUpperLimit()) elevatorSpeed = speed;
  }

  public void zeroEncoder() {
    elevatorEncoder.setPosition(0);
  }

  public void up() {
    if (!atLowerLimit() && !atUpperLimit()) elevatorSpeed = ElevatorConstants.upSpeed;
  }

  public void down() {
    if (!atLowerLimit() && !atUpperLimit()) elevatorSpeed = ElevatorConstants.downSpeed;
  }

  public void stop() {
    elevatorSpeed = 0;
  }

  private void configMotorControllerDefaults() {
    elevatorMotorConfig.inverted(false);
    elevatorMotorConfig.idleMode(IdleMode.kBrake);
    elevatorMotorConfig.encoder.positionConversionFactor(ElevatorConstants.conversionFactor);

    configMotorcontroller();
  }

  private void configMotorcontroller() {
    elevatorMotor.configure(elevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public double getEncoderVal() {
    return elevatorEncoder.getPosition();
  }

  public void lockElevator() {
    elevatorSpeed = ElevatorConstants.lockSpeed;
  }

  public boolean atUpperLimit() {
    return getEncoderVal() >= ElevatorConstants.upperLimit;
  }

  public boolean atLowerLimit() {
    return getEncoderVal() <= ElevatorConstants.lowerLimit;
  }
}
