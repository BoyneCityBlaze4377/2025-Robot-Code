package frc.robot.subsystems;

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

public class AlgaeAffector extends SubsystemBase {
  private final SparkMax algaeCollectorOne, algaeCollectorTwo;
  private final SparkMaxConfig algaeCollectorOneConfig, algaeCollectorTwoConfig;
  private final DigitalInput algaeDetector;

  /** Creates a new PieceAffector. */
  public AlgaeAffector() {
    algaeCollectorOne = new SparkMax(AffectorConstants.algaeCollectorOneID, MotorType.kBrushless);
    algaeCollectorTwo = new SparkMax(AffectorConstants.algaeCollectorTwoID, MotorType.kBrushless);

    algaeCollectorOneConfig = new SparkMaxConfig();
    algaeCollectorTwoConfig = new SparkMaxConfig();

    configMotorControllerDefaults();

    algaeDetector = new DigitalInput(SensorConstants.algaeBreakID);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //IOConstants.MatchTab.add("HasAlgae", hasAlgae());
  }

  private void configMotorControllerDefaults() {
    algaeCollectorOneConfig.inverted(true);
    algaeCollectorOneConfig.idleMode(IdleMode.kCoast);

    algaeCollectorTwoConfig.inverted(true);
    algaeCollectorTwoConfig.idleMode(IdleMode.kCoast);

    configMotorControllers();
  }

  private void configMotorControllers() {
    algaeCollectorOne.configure(algaeCollectorOneConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    algaeCollectorTwo.configure(algaeCollectorTwoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void collect() {
    algaeCollectorOne.set(-.2);
    algaeCollectorTwo.set(.2);
  }

  public void eject() {
    algaeCollectorOne.set(.2);
    algaeCollectorTwo.set(-.2);
  }

  public void stopAffector() {
    algaeCollectorOne.set(0);
    algaeCollectorTwo.set(0);
  }

  public boolean getDetector() {
    return algaeDetector.get();
  }

  public boolean hasAlgae() {
    return !algaeDetector.get();
  }
}
