package frc.robot;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;

import frc.robot.Constants.MotorConstants;


public class Configs {
  public static final class IntakeConfigs {
    // Intake Kraken x60
    public static final TalonFXConfiguration INTAKE_TALON_FX_CONFIGURATION = new TalonFXConfiguration();
    public static final TalonFXConfiguration CENTERER_TALON_FX_CONFIGURATION = new TalonFXConfiguration();
    static{
    /*
      ********************************************
      **    INTAKE KRAKEN x60 CONFIGURATIONS    **
      ********************************************
    */

    INTAKE_TALON_FX_CONFIGURATION.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = MotorConstants.k_intakeRampRate;
    INTAKE_TALON_FX_CONFIGURATION.MotorOutput.PeakForwardDutyCycle = MotorConstants.k_intakeClosedMaxSpeed;
    INTAKE_TALON_FX_CONFIGURATION.MotorOutput.PeakReverseDutyCycle = -MotorConstants.k_intakeClosedMaxSpeed;
    INTAKE_TALON_FX_CONFIGURATION.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    INTAKE_TALON_FX_CONFIGURATION.CurrentLimits.SupplyCurrentLimit = MotorConstants.k_intakeSupplyCurrentLimit;
    /*
      ********************************************
      **    CENTERER KRAKEN x60 CONFIGURATIONS    **
      ********************************************
    */

    CENTERER_TALON_FX_CONFIGURATION.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = MotorConstants.k_centererRampRate;
    CENTERER_TALON_FX_CONFIGURATION.MotorOutput.PeakForwardDutyCycle = MotorConstants.k_centererClosedMaxSpeed;
    CENTERER_TALON_FX_CONFIGURATION.MotorOutput.PeakReverseDutyCycle = -MotorConstants.k_centererClosedMaxSpeed;
    CENTERER_TALON_FX_CONFIGURATION.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    CENTERER_TALON_FX_CONFIGURATION.CurrentLimits.SupplyCurrentLimit = MotorConstants.k_centererSupplyCurrentLimit;  
  }
}
  
  public static final class ShootConfigs {
  // Shooter Kraken x60
  public static final TalonFXConfiguration SHOOT_TALON_FX_CONFIGURATION = new TalonFXConfiguration();

  // Conveyor Kraken x44
  public static final TalonFXConfiguration CONVEYOR_TALON_FX_CONFIGURATION = new TalonFXConfiguration();

  // Conveyor CANRange
  public static final CANrangeConfiguration SHOOT_CANRANGE_CONFIGURATION = new CANrangeConfiguration();
  static{
  /*
      ********************************************
      **    SHOOT KRAKEN x60 CONFIGURATIONS    **
      ********************************************
    */

    SHOOT_TALON_FX_CONFIGURATION.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = MotorConstants.k_shooterRampRate;
    SHOOT_TALON_FX_CONFIGURATION.MotorOutput.PeakForwardDutyCycle = MotorConstants.k_shooterClosedMaxSpeed;
    SHOOT_TALON_FX_CONFIGURATION.MotorOutput.PeakReverseDutyCycle = -MotorConstants.k_shooterClosedMaxSpeed;
    SHOOT_TALON_FX_CONFIGURATION.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    SHOOT_TALON_FX_CONFIGURATION.CurrentLimits.SupplyCurrentLimit = MotorConstants.k_shooterSupplyCurrentLimit;

    /*
      ********************************************
      **    CONVEYOR KRAKEN x44 CONFIGURATIONS    **
      ********************************************
    */

    CONVEYOR_TALON_FX_CONFIGURATION.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = MotorConstants.k_conveyorRampRate;
    CONVEYOR_TALON_FX_CONFIGURATION.MotorOutput.PeakForwardDutyCycle = MotorConstants.k_conveyorClosedMaxSpeed;
    CONVEYOR_TALON_FX_CONFIGURATION.MotorOutput.PeakReverseDutyCycle = -MotorConstants.k_conveyorClosedMaxSpeed;
    CONVEYOR_TALON_FX_CONFIGURATION.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    CONVEYOR_TALON_FX_CONFIGURATION.CurrentLimits.SupplyCurrentLimit = MotorConstants.k_conveyorSupplyCurrentLimit;

      /*
      ******************************************
      **    SHOOT CANRANGE CONFIGURATIONS    **
      ******************************************
    */

    SHOOT_CANRANGE_CONFIGURATION.ProximityParams.MinSignalStrengthForValidMeasurement = 2500; // TODO: Make it bigger?
    SHOOT_CANRANGE_CONFIGURATION.ProximityParams.ProximityThreshold = 0.1; // TODO: Tune for status Lights
    SHOOT_CANRANGE_CONFIGURATION.ProximityParams.ProximityHysteresis = 0.01;
    
    SHOOT_CANRANGE_CONFIGURATION.ToFParams.UpdateFrequency = 50;
    SHOOT_CANRANGE_CONFIGURATION.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;   
  }
} 
}

