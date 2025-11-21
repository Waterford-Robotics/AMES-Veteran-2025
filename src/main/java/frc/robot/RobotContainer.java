// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.CANRangeSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  public final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
  public final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  public final ShootSubsystem m_shooterSubsystem = new ShootSubsystem();
  private final CANRangeSubsystem m_canRangeSubsystem = new CANRangeSubsystem();
  
  private final CommandXboxController m_driveController = new CommandXboxController(ControllerConstants.k_driverControllerPort);
  //private final CommandXboxController m_operatorController = new CommandXboxController(ControllerConstants.k_operatorControllerPort);

  public RobotContainer() {
    configureBindings();

    m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity);
  }

  private void configureBindings() {
    //intake right trigger
    new Trigger(() -> m_driveController.getRawAxis(ControllerConstants.k_righttrig) > 0.05)
      .whileTrue(
        new InstantCommand(() -> m_intakeSubsystem.intake(), m_intakeSubsystem))
      .whileTrue(
        new InstantCommand(() -> m_intakeSubsystem.centerer(), m_intakeSubsystem)
        .until(() -> m_canRangeSubsystem.getIsDetected()))
      .whileTrue(
        new InstantCommand(() -> m_shooterSubsystem.conveyor(), m_shooterSubsystem)
        .until(() -> m_canRangeSubsystem.getIsDetected()))
      .onFalse(
        new InstantCommand(() -> m_intakeSubsystem.stopIntake(), m_intakeSubsystem)
      )
      .onFalse(
        new InstantCommand(() -> m_shooterSubsystem.stopConveyor(), m_shooterSubsystem));

    //spin up left trigger
    new Trigger(() -> m_driveController.getRawAxis(ControllerConstants.k_lefttrig)>0.05)
      .whileTrue(
        new InstantCommand(() -> m_shooterSubsystem.spinUp(), m_shooterSubsystem))
      .onFalse(
        new InstantCommand(() -> m_shooterSubsystem.stopShooter(), m_shooterSubsystem)
      );

      //shoot right bumper
      new JoystickButton(m_driveController.getHID(), ControllerConstants.k_rightbump)
      .onTrue(
        new InstantCommand(() -> m_intakeSubsystem.intake(), m_intakeSubsystem)
      )
      .onTrue(
        new InstantCommand(() -> m_intakeSubsystem.centerer(), m_intakeSubsystem)
      )
      .onTrue(
        new InstantCommand(() -> m_shooterSubsystem.shoot(), m_shooterSubsystem)
      )
      .onFalse(
        new InstantCommand(() -> m_shooterSubsystem.stopShooter(), m_shooterSubsystem))
      .onFalse(
        new InstantCommand(() -> m_shooterSubsystem.stopConveyor(), m_shooterSubsystem))
      .onFalse(
        new InstantCommand(() -> m_intakeSubsystem.stopIntake(), m_shooterSubsystem));
  }

  public Command driveFieldOrientedAngularVelocity = m_swerveSubsystem.driveCommand(
    () -> MathUtil.applyDeadband(m_driveController.getLeftY() * DriveConstants.k_driveSpeed, DriveConstants.k_driveDeadBand),
    () -> MathUtil.applyDeadband(m_driveController.getLeftX() * DriveConstants.k_driveSpeed, DriveConstants.k_driveDeadBand),
    () -> m_driveController.getRightX() * DriveConstants.k_turnRate);

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
