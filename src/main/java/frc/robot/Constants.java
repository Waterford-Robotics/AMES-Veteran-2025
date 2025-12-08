package frc.robot;

import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;

public class Constants {
      // Constants for Kraken Drivetrain!
  public static final class SwerveConstants {

    // Must be max physically possible speed
    public static final double k_maxSpeed = edu.wpi.first.math.util.Units.feetToMeters(18.9); // Meters per second
    public static final double k_maxAngularSpeed = 1.5 * Math.PI; // Radians per second
  }

  public static final class ControllerConstants {
    public static final int k_driverControllerPort = 0; // Driver
    public static final int k_operatorControllerPort = 1; // Operator

    // public static final double k_driveDeadband = 0.10; // Increase to combat larger stick drift

    public final static int k_start = Button.kStart.value; // Start Button
    public final static int k_back = Button.kBack.value; // Back Button

    public static final int k_A = Button.kA.value; // A
    public static final int k_B = Button.kB.value; // B
    public static final int k_X = Button.kX.value; // X
    public static final int k_Y = Button.kY.value; // Y
    
    public static final int k_dpadup = 0; // D-Pad Up
    public static final int k_dpadRight = 90; // D-Pad Right
    public static final int k_dpadDown = 180; // D-Pad Down
    public static final int k_dpadLeft = 270; // D-Pad Left

    public final static int k_rightbump = Button.kRightBumper.value; // Right Bump
    public final static int k_leftbump = Button.kLeftBumper.value; // Left Bump

    public final static int k_righttrig = Axis.kRightTrigger.value; // Right Trig
    public final static int k_lefttrig = Axis.kLeftTrigger.value; // Left Trig
  }  

  public static final class DriveConstants{
    public static final double k_driveDeadBand = 0.1;
    public static final double k_driveSpeed = -0.8;
    public static final double k_turnRate = -0.85;
  }
  
  public static final class MotorConstants{
    public static final int k_supplyCurrentLimit = 40;

    //intake
    public static final int k_intakeKrakenID = 12;
    public static final double k_intakeRampRate = 0.05;
    public static final double k_intakeClosedMaxSpeed = 0.4;
    public static final int k_intakeSupplyCurrentLimit = 60;
    public static final double k_intakeSpeed = -0.5;

    //centerer
    public static final int k_centererKrakenID = 13;
    public static final double k_centererRampRate = 0.05;
    public static final double k_centererClosedMaxSpeed = 0.4;
    public static final int k_centererSupplyCurrentLimit = 60;
    public static final double k_centererSpeed = 0.1;

    //conveyor intake
    public static final int k_conveyorKrakenID = 11;
    public static final double k_conveyorRampRate = 0.05;
    public static final double k_conveyorClosedMaxSpeed = 0.4;
    public static final int k_conveyorSupplyCurrentLimit = 60;
    public static final double k_conveyorSpeed = 0.12; 

    //shooter
    public static final int k_shooterKrakenID = 21;
    public static final double k_shooterRampRate = 0.05;
    public static final double k_shooterClosedMaxSpeed = 0.4;
    public static final int k_shooterSupplyCurrentLimit = 60;
    public static final double k_shooterSpeed = 0.31;
  }
  
  public static final class SensorIDConstants {
    // Intake CANRange
    public static final int k_shootCANRangeID = 42;
  }
}
