package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.ShootConfigs;
import frc.robot.Constants.SensorIDConstants;
public class CANRangeSubsystem extends SubsystemBase{
  
  private CANrange m_canrange;

  public CANRangeSubsystem() {
    m_canrange = new CANrange(SensorIDConstants.k_shootCANRangeID, "rio");
    m_canrange.getConfigurator().apply(ShootConfigs.SHOOT_CANRANGE_CONFIGURATION, 0.05);
  }

  public boolean getIsDetected() {
    return m_canrange.getIsDetected().getValue();
}

  public void periodic() {
    SmartDashboard.putBoolean("canrange", getIsDetected());
  }
}