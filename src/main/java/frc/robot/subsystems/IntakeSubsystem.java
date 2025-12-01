
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.IntakeConfigs;
import frc.robot.Constants.MotorConstants;

public class IntakeSubsystem extends SubsystemBase{

  private TalonFX m_intake;
  private TalonFX m_centerer;

  public IntakeSubsystem() {
    m_intake = new TalonFX(MotorConstants.k_intakeKrakenID);
    m_intake.getConfigurator().apply(IntakeConfigs.INTAKE_TALON_FX_CONFIGURATION, 0.05);

    m_centerer = new TalonFX(MotorConstants.k_centererKrakenID);
    m_centerer.getConfigurator().apply(IntakeConfigs.CENTERER_TALON_FX_CONFIGURATION, 0.05);
  }

  public void intake() {
    m_intake.set(MotorConstants.k_intakeSpeed);
  }

  public void centerer() {
    m_centerer.set(MotorConstants.k_centererSpeed);
  }

  public void intakeSpeed(double power) {
    m_intake.set(power);
  }

  public void stopIntake() {
    m_intake.set(0);
  }
  public void stopCenterer() {
    m_centerer.set(0);
  }

  public void periodic() {
  }
}
