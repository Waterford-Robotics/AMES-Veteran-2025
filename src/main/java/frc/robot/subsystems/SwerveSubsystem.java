// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

// Swerve Subsystem Code yippee
public class SwerveSubsystem extends SubsystemBase {

  // Imports stuff from the JSON Files
  File directory = new File(Filesystem.getDeployDirectory(),"swerve");
  public static SwerveDrive swerveDrive;

  // Red Alliance sees forward as 180 degrees, Blue Alliance sees as 0
  public static int AllianceYaw;

  // Field2d
  private Field2d m_field = new Field2d();

  private RobotConfig config;

  // Creates a New SwerveSubsystem
  public SwerveSubsystem() {

    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
    // TURN OFF DURING COMPETITION BECAUSE IT * WILL *  SLOW YOUR ROBOT (It's for displaying info in Shuffleboard)
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.LOW;
    
    // Initializes robot using the JSON Files with all the constants so you don't have to. Hooray!
    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(SwerveConstants.k_maxSpeed);
    } 
    catch (Exception e) {
      throw new RuntimeException(e);
    }
    
    // Now you have to it manually (aww)
    swerveDrive.pushOffsetsToEncoders();

    // Cosine Compensator makes your robot slower on some wheels. Set it to opposite bool if it drives funky
    swerveDrive.setCosineCompensator(false);

    // Keeps robot locked in position when moving, keep false
    swerveDrive.setHeadingCorrection(false);

    // Field on Smart Dashboard
    SmartDashboard.putData("Field", m_field);

    try{
      config = RobotConfig.fromGUISettings();
    } 
    catch (Exception e) {
      e.printStackTrace();
    }

    AutoBuilder.configure( 
      this::getPose, // Robot pose supplier
      this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
        new PIDConstants(0.0, 0.0, 0.0) // Rotation PID constants
      ),
      config,
      () -> {

        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {  
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this // Reference to this subsystem to set requirements
    );

  }

  // This method will be called once per scheduler run
  // Periodically update the odometry w/vision
  public void periodic() {

    // Updates Odometry with Vision if Applicable
    // updateVisionMeasurements();

    // Update Field2d
    m_field.setRobotPose(swerveDrive.getPose());

    // Puts position of robot on smartdashboard
    SmartDashboard.putNumber("X Position", swerveDrive.getPose().getX());
    SmartDashboard.putNumber("Y Position", swerveDrive.getPose().getY());
  }

  // Command to drive the robot using translative values and heading as angular velocity.
  // translationX - Translation in the X direction. Cubed for smoother controls.
  // translationY - Translation in the Y direction. Cubed for smoother controls.
  // angularRotationX - Angular velocity of the robot to set. Cubed for smoother controls.
  // Returns Drive command.

  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
    return run(() -> {
      // Make the robot move
      swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
        translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
        translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), 0.8),
        Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
        true,
        false);
    });
  }



  // Command to drive the robot using translative values and heading as a setpoint.
  // translationX Translation in the X direction. Cubed for smoother controls.
  // translationY Translation in the Y direction. Cubed for smoother controls.
  // headingX     Heading X to calculate angle of the joystick.
  // headingY     Heading Y to calculate angle of the joystick.
  // returns Drive command.
  // I'm kinda useless btw, thanks for reading
  
  public Command driveCommandTranslative(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX, DoubleSupplier headingY) {
    return run(() -> {
      Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()), 0.8);

      // Make the robot move
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(
        scaledInputs.getX(), scaledInputs.getY(),
        headingX.getAsDouble(), headingY.getAsDouble(),
        swerveDrive.getOdometryHeading().getRadians(),
        swerveDrive.getMaximumChassisVelocity()
      ));
    });
  }

  // Gets the current pose (position and rotation) of the robot, as reported by odometry.
  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  // Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
  // method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
  // keep working.
  public void resetOdometry(Pose2d initialHolonomicPose) {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  // Gets the current velocity (x, y and omega) of the robot
  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  // Set chassis speeds with closed-loop velocity control.
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  // Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  // Gets robot yaw
  public Rotation2d getYaw() {
    return swerveDrive.getYaw();
  }

  // Gets robot rotation rate in degrees
  public double getRotationRateDegrees() {
    return swerveDrive.getGyro().getYawAngularVelocity().magnitude();
  }

  // Sets X Pose to modules
  // What did YAGSL say to the modules?
  // "Lock in"
  public void setx() {
    swerveDrive.lockPose();
  }

  // Drive Field Oriented With ChassisSpeeds
  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    swerveDrive.driveFieldOriented(velocity);
  }

  // Get Auto Command
  public Command getAutonomousCommand(String pathName) {
    return new PathPlannerAuto(pathName);
  }

  // Returns the swerve drive
  public static SwerveDrive getInstance() {
    return swerveDrive;
  }
}