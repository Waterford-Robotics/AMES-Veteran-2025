// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.I2C;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
// import edu.wpi.first.wpilibj2.command.Command;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

public class ColorSensorSubsystem extends SubsystemBase {
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private  Color kBlueTarget = new Color("#1434A4");//#1434A4    also removed final key word, if needed add back
  private  Color kGreenTarget = new Color("#008000");//#008000
  private  Color kRedTarget = new Color("#D40000");//#D40000
  private  Color kYellowTarget = new Color("#FFFF00");//#FFFF00
  private  Color kPurpleTarget = new Color("#9600B0");//#9600B0
  private  Color kOrangeTarget = new Color("#FF7B00");//#FF7B00
  double averageR, averageG, averageB;
  int tickingForAvg=0;
  public String colorString = new String();

  public ColorSensorSubsystem() {
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);   
    m_colorMatcher.addColorMatch(kPurpleTarget);
    m_colorMatcher.addColorMatch(kOrangeTarget);    
  }

  public Color getDetectedColor(){
    return m_colorSensor.getColor();
  }
  public ColorMatchResult getClosestColor(){
    return m_colorMatcher.matchClosestColor(this.getDetectedColor());
  }


  @Override
  public void periodic() {
    if (this.getClosestColor().color == kBlueTarget) {
      colorString = "Blue";
    } else if (this.getClosestColor().color == kRedTarget) {
      colorString = "Red";
    } else if (this.getClosestColor().color == kGreenTarget) {
      colorString = "Green";
    } else if (this.getClosestColor().color == kYellowTarget) {
      colorString = "Yellow";
    }else if (this.getClosestColor().color == kPurpleTarget) {
      colorString = "Purple";
    }else if (this.getClosestColor().color == kOrangeTarget) {
      colorString = "Orange";
    }else {
      colorString = "Unknown";
    }

    if(tickingForAvg==0){
      averageR = this.getClosestColor().color.red;
      averageG = this.getClosestColor().color.green;
      averageB = this.getClosestColor().color.blue;

      m_colorMatcher.addColorMatch(kBlueTarget);
      m_colorMatcher.addColorMatch(kGreenTarget);
      m_colorMatcher.addColorMatch(kRedTarget);
      m_colorMatcher.addColorMatch(kYellowTarget);   
      m_colorMatcher.addColorMatch(kPurpleTarget);
      m_colorMatcher.addColorMatch(kOrangeTarget);
    }else if(tickingForAvg==1){
      averageR=(averageR+this.getClosestColor().color.red)/2;
      averageG=(averageG+this.getClosestColor().color.green)/2;
      averageB=(averageB+this.getClosestColor().color.blue)/2;
    }else if(tickingForAvg==3){
      tickingForAvg=1;
    }
    tickingForAvg++;

    // if(controller.getRawButton(4)){
    //   kRedTarget = new Color(averageR, averageG, averageB);
    // }

    SmartDashboard.putNumber("Red", this.getClosestColor().color.red);
    SmartDashboard.putNumber("Green", this.getClosestColor().color.green);
    SmartDashboard.putNumber("Blue", this.getClosestColor().color.blue);

    String confidence;
    if(this.getClosestColor().confidence<0.25){
      confidence = new String("not confident");
    }else if(this.getClosestColor().confidence<0.5){
      confidence = new String("not very confident");
    }else if(this.getClosestColor().confidence<0.75){
      confidence = new String("decently confident");
    }else{
      confidence = new String("very confident");
    }
    SmartDashboard.putString("Confidence", confidence);

    SmartDashboard.putString("Detected Color", colorString);

    SmartDashboard.putNumber("AvgR", averageR);    
    SmartDashboard.putNumber("AvgG", averageG);
    SmartDashboard.putNumber("AvgB", averageB);
  }
}
