// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.CANRangeSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.WoodSubsystem;

import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {
  public final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
  public final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  public final ShootSubsystem m_shooterSubsystem = new ShootSubsystem();
  public final WoodSubsystem m_woodSubsystem = new WoodSubsystem();
  private final CANRangeSubsystem m_canRangeSubsystem = new CANRangeSubsystem();
  private final LEDSubsystem m_LedSubsystem = new LEDSubsystem();

  private final CommandXboxController m_driveController = new CommandXboxController(ControllerConstants.k_driverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(ControllerConstants.k_operatorControllerPort);

  double k_autoWait;
  SendableChooser<Command> m_chooser = new SendableChooser<Command>();

  public RobotContainer() {
    configureBindings();
    m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity);
    NamedCommands.registerCommand("Rev", AutoRevCommand);
    NamedCommands.registerCommand("Shoot", AutoShootCommand);
    NamedCommands.registerCommand("Halt Shoot", AutoShootHaltCommand);

    k_autoWait = SmartDashboard.getNumber("Auto Wait Time", 0);
    m_chooser.addOption("Triple Shoot Short", m_swerveSubsystem.getAutonomousCommand("Triple Shoot Short"));
    m_chooser.addOption("Triple Shoot Long", m_swerveSubsystem.getAutonomousCommand("Triple Shoot Long"));
  }

  private void configureBindings() {
    //intake, conveyor, centerer — right trigger
    new Trigger(() -> m_driveController.getRawAxis(ControllerConstants.k_righttrig) > 0.05)
      .whileTrue(
        new InstantCommand(() -> m_intakeSubsystem.intake(), m_intakeSubsystem)
        )
      .whileTrue(
        new IntakeCommand(m_shooterSubsystem, m_canRangeSubsystem)
        ) 
      .whileTrue(
        new InstantCommand(() -> m_intakeSubsystem.centerer(), m_intakeSubsystem)
      )
      .onFalse(
        new InstantCommand(() -> m_intakeSubsystem.stopIntake(), m_intakeSubsystem)
        )
      .onFalse(
        new InstantCommand(() -> m_intakeSubsystem.stopCenterer(), m_intakeSubsystem)
        )
      .onFalse(
        new InstantCommand(() -> m_shooterSubsystem.stopConveyor(), m_shooterSubsystem)
    );
    
    // unintake run in other direction — A
    new JoystickButton(m_driveController.getHID(), ControllerConstants.k_A)
      .onTrue(
      new InstantCommand(() -> m_intakeSubsystem.anti(), m_shooterSubsystem))
      .onFalse(
        new InstantCommand(() -> m_intakeSubsystem.stopIntake(), m_intakeSubsystem))
      .onFalse(
        new InstantCommand(() -> m_intakeSubsystem.stopCenterer(), m_intakeSubsystem)
    );
    // Wood Shooter B and  
    new JoystickButton(m_driveController.getHID(), ControllerConstants.k_B)
      .onTrue(
        new InstantCommand(()-> m_woodSubsystem.runWood(1), m_woodSubsystem))
      .onFalse(
        new InstantCommand(()-> m_woodSubsystem.stopWood(), m_woodSubsystem)
      );

      new JoystickButton(m_driveController.getHID(), ControllerConstants.k_Y)
      .onTrue(
        new InstantCommand(()-> m_woodSubsystem.runWood(-1), m_woodSubsystem))
      .onFalse(
        new InstantCommand(()-> m_woodSubsystem.stopWood(), m_woodSubsystem)
      );

    //spin up — left bumper (operator)
    new JoystickButton(m_driveController.getHID(), ControllerConstants.k_leftbump)
      .whileTrue(
        new InstantCommand(() -> m_shooterSubsystem.spinUp(), m_shooterSubsystem)
      )
      .onFalse(
        new InstantCommand(() -> m_shooterSubsystem.stopShooter(), m_shooterSubsystem)
    );

    //shoot — right bumper (operator)
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
        new InstantCommand(() -> m_shooterSubsystem.stopShooter(), m_shooterSubsystem)
      )
      .onFalse(
        new InstantCommand(() -> m_shooterSubsystem.stopConveyor(), m_shooterSubsystem)
      )
      .onFalse(
        new InstantCommand(() -> m_intakeSubsystem.stopIntake(), m_shooterSubsystem)
    )
    .onFalse(
      new InstantCommand(() -> m_intakeSubsystem.stopCenterer(), m_intakeSubsystem)
    );


    // Reset Gyro - X
    new JoystickButton(m_driveController.getHID(), ControllerConstants.k_X)
      .onTrue(
        new InstantCommand(() -> m_swerveSubsystem.zeroGyro(), m_swerveSubsystem)
    );
  }

  public Command driveFieldOrientedAngularVelocity = m_swerveSubsystem.driveCommand(
    () -> MathUtil.applyDeadband(m_driveController.getLeftY() * DriveConstants.k_driveSpeed, DriveConstants.k_driveDeadBand),
    () -> MathUtil.applyDeadband(m_driveController.getLeftX() * DriveConstants.k_driveSpeed, DriveConstants.k_driveDeadBand),
    () -> m_driveController.getRightX() * DriveConstants.k_turnRate);


  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
      new WaitCommand(k_autoWait),
      m_swerveSubsystem.getAutonomousCommand("TripleShoot Short")
    );
  }

  SequentialCommandGroup AutoRevCommand = new SequentialCommandGroup(
    new InstantCommand(() -> m_shooterSubsystem.spinUp(), m_shooterSubsystem)
  );

  SequentialCommandGroup AutoShootCommand = new SequentialCommandGroup(
    new InstantCommand(() -> m_intakeSubsystem.intake(), m_intakeSubsystem),
    new InstantCommand(() -> m_intakeSubsystem.centerer(), m_intakeSubsystem),
    new InstantCommand(() -> m_shooterSubsystem.shoot(), m_shooterSubsystem)
  );

  SequentialCommandGroup AutoShootHaltCommand = new SequentialCommandGroup(
    new InstantCommand(() -> m_shooterSubsystem.stopShooter(), m_shooterSubsystem),
    new InstantCommand(() -> m_shooterSubsystem.stopConveyor(), m_shooterSubsystem),
    new InstantCommand(() -> m_intakeSubsystem.stopIntake(), m_shooterSubsystem),
    new InstantCommand(() -> m_intakeSubsystem.stopCenterer(), m_intakeSubsystem)
  );
}
