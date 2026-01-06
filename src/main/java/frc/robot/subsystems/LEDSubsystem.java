package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    Spark m_blinkin;
    ColorSensorSubsystem m_sensor = new ColorSensorSubsystem();

    public LEDSubsystem() {
        //enter port constant
        m_blinkin = new Spark(1);
    }
    
    
    //colors

    public void setRed() {
        m_blinkin.set(0.61);
    }

    public void setOrange() {
        m_blinkin.set(0.65);
    }

    public void setYellow() {
        m_blinkin.set(0.69);
    }

    public void setGreen() {
        m_blinkin.set(0.77);
    }

    public void setBlue() {
        m_blinkin.set(0.87);
    }

    public void setPurple() {
        m_blinkin.set(0.91);
    }

    public void setFireMedium() {
        m_blinkin.set(-0.59);
    }

    public void turnOff() {
        m_blinkin.set(0.00);
    }


    //need color sensor subsystem stuffs

    public void periodic() {
       String color = m_sensor.colorString;
       
       if (color.equals("Red")) {
        setRed();
       } 
       else if (color.equals("Orange")) {
        setOrange();
       } 
       else if (color.equals("Yellow")) {
        setYellow();
       }
       else if (color.equals("Green")) {
        setGreen();
       }
       else if (color.equals("Blue")) {
        setBlue();
       }
       else if (color.equals("Purple")) {
        setPurple();
       }
       else if(color.equals("Empty")){
        setFireMedium();
       }
       else {
        turnOff();
       }

    }
}