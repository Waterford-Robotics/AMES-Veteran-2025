package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.ShootConfigs;
import frc.robot.Constants.MotorConstants;

public class ShootSubsystem extends SubsystemBase{

  private TalonFX m_conveyor;
  private TalonFX m_shooter;

  public ShootSubsystem() {
    
    m_conveyor = new TalonFX(MotorConstants.k_conveyorKrakenID);
    m_shooter = new TalonFX(MotorConstants.k_shooterKrakenID);

    m_shooter.getConfigurator().apply(ShootConfigs.SHOOT_TALON_FX_CONFIGURATION, 0.05);

    m_conveyor.getConfigurator().apply(ShootConfigs.CONVEYOR_TALON_FX_CONFIGURATION, 0.05);
  }

  public void spinUp(){
    m_shooter.set(MotorConstants.k_shooterSpeed);
  }
  public void shoot() {
    m_conveyor.set(MotorConstants.k_conveyorSpeed);
    m_shooter.set(MotorConstants.k_shooterSpeed);
  }

  public void stopShooter() {
    m_shooter.set(0);
    m_conveyor.set(0);
  }

  public void periodic() {
  }
}
