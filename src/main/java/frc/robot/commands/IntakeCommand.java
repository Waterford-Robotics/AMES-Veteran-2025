// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.CANRangeSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCommand extends Command {

  // Uses Elevator and Subsystems

  ShootSubsystem m_shootSubsystem;
  CANRangeSubsystem m_canRangeSubsystem;

  // Constructor
  public IntakeCommand(ShootSubsystem shootSubsystem, CANRangeSubsystem canRangeSubsystem) {
        
    // Definitions and setting parameters are equal to members!

    m_shootSubsystem = shootSubsystem;
    addRequirements(shootSubsystem);

    m_canRangeSubsystem = canRangeSubsystem;
    addRequirements(canRangeSubsystem);
  }

  public void initialize() {}
  
  // run the centerer and conveyor until canrange detects
  public void execute() {
    if(!m_canRangeSubsystem.getIsDetected()) {
      m_shootSubsystem.conveyor();
    }
  }

  // when the command is over stop running the centerer and conveyors
  public void end(boolean interrupted) {
    m_shootSubsystem.stopConveyor();
  }

  // Checks if the command is done
  public boolean isFinished() {
    return m_canRangeSubsystem.getIsDetected();
  }
}
