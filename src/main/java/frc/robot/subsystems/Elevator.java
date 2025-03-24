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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.AutoAimConstants.Position;

public class Elevator extends SubsystemBase {
  private final SparkMax elevatorMotor;
  private final PIDController elevatorController;
  private final SparkMaxConfig elevatorMotorConfig;
  private final RelativeEncoder elevatorEncoder;
  private boolean locked, atPos;
  private double elevatorSpeed;
  private String positionStatusString;
  private final double dH;
  private Position currentPosition;

  private final GenericEntry elevatorHeight, elevatorSpeedSender, upperLimit, lowerLimit, 
                             positionStatusSender, lockedSender, atPositionSender;

  /** Creates a new Elevator. */
  public Elevator() {
    elevatorMotor = new SparkMax(ElevatorConstants.elevatorMotorID, MotorType.kBrushless);

    elevatorMotorConfig = new SparkMaxConfig();

    elevatorEncoder = elevatorMotor.getEncoder();
    elevatorController = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
    elevatorController.setTolerance(ElevatorConstants.kTolerance);

    elevatorSpeed = 0;
    dH = ElevatorConstants.startingHeight - elevatorEncoder.getPosition();

    positionStatusString = "At floor";
    currentPosition = Position.floor;

    atPos = false;
    locked = false;

    configMotorControllerDefaults();
    elevatorHeight = IOConstants.TeleopTab.add("Elevator Height", 0)
                                         .withWidget("Number Bar")
                                         .withProperties(Map.of("min_value", 0,
                                                                "max_value", 220,
                                                                "divisions", 11,
                                                                "orientation", "vertical"))
                                         .getEntry();
    elevatorSpeedSender = IOConstants.DiagnosticTab.add("Elevator Speed", 0)
                                                   .withWidget("Number Slider")
                                                   .withProperties(Map.of("min_value", -1, "max_value", 1))
                                                   .getEntry();
    upperLimit = IOConstants.DiagnosticTab.add("At Upper Limit?", false)
                                          .withWidget("Boolean Box").getEntry();
    lowerLimit = IOConstants.DiagnosticTab.add("At Lower Limit?", true)
                                          .withWidget("Boolean Box").getEntry();
    positionStatusSender = IOConstants.TeleopTab.add("Position", positionStatusString)
                                               .withWidget("Text Display").getEntry();
    lockedSender = IOConstants.DiagnosticTab.add("Elevator locked", false)
                                            .withWidget("Boolean Box").getEntry();
    atPositionSender = IOConstants.TeleopTab.add("At Position?", atPos)
                                            .withWidget("Boolean Box").getEntry();
  }

  @Override
  public void periodic() {
    // if (atUpperLimit()) {
    //   elevatorMotor.set(-ElevatorConstants.correctionSpeed);
    // } else if (atLowerLimit()) {
    //   elevatorMotor.set(ElevatorConstants.correctionSpeed);
    // } else {
    //   elevatorMotor.set(elevatorSpeed);
    // }
    
    elevatorHeight.setDouble(getEncoderVal());
    elevatorSpeedSender.setDouble(elevatorSpeed);
    upperLimit.setBoolean(atUpperLimit());
    lowerLimit.setBoolean(atLowerLimit());
    positionStatusSender.setString(positionStatusString);
    lockedSender.setBoolean(locked);
    atPositionSender.setBoolean(atPos);
  }

  public void set(double speed) {
    if (!atLowerLimit() && !atUpperLimit()) elevatorSpeed = speed;
    locked = false;
  }

  public void zeroEncoder() {
    elevatorEncoder.setPosition(0);
  }

  public void up() {
    if (!atLowerLimit() && !atUpperLimit()) elevatorSpeed = ElevatorConstants.upSpeed;
    locked = false;
  }

  public void down() {
    if (!atLowerLimit() && !atUpperLimit()) elevatorSpeed = ElevatorConstants.downSpeed;
    locked = false;
  }

  public void stop() {
    elevatorSpeed = 0;
    locked = false;
  }

  private void configMotorControllerDefaults() {
    elevatorMotorConfig.inverted(false);
    elevatorMotorConfig.idleMode(IdleMode.kBrake);
    elevatorMotorConfig.encoder.positionConversionFactor(ElevatorConstants.conversionFactor);

    elevatorMotor.configure(elevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public double getEncoderVal() {
    return elevatorEncoder.getPosition() + dH;
  }

  public void lockElevator() {
    elevatorSpeed = ElevatorConstants.lockSpeed;
    locked = true;
  }

  public boolean atUpperLimit() {
    return getEncoderVal() >= ElevatorConstants.upperLimit;
  }

  public boolean atLowerLimit() {
    return getEncoderVal() <= ElevatorConstants.lowerLimit;
  }

  public void setPositionString(String positionString) {
    positionStatusString = positionString;
  }

  public void setAtPos(boolean atPosition) {
    atPos = atPosition;
  }

  public void setSetpoint(double setPoint) {
    elevatorController.setSetpoint(setPoint);
  }

  public void PIDMove() {
    elevatorSpeed = MathUtil.clamp(elevatorController.calculate(getEncoderVal()), 
                    ElevatorConstants.maxDownSpeed, ElevatorConstants.maxUpSpeed);
    locked = false;
  }

  public boolean atSetpoint() {
    return elevatorController.atSetpoint();
  }

  public void setCurrentPosition(Position position) {
    currentPosition = position;
  }

  public Position getCurrentPosition() {
    return currentPosition;
  }
}
