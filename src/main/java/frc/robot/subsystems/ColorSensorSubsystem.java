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
  private  Color kBlueTarget = new Color("#266373");//#1434A4    changed                also removed final key word, if needed add back
  private  Color kGreenTarget = new Color("#309C30");//#008000   changed
  private  Color kRedTarget = new Color("#8F5717");//#D40000     changed
  private  Color kYellowTarget = new Color("#4d9c12");//#FFFF00  changed
  private  Color kPurpleTarget = new Color("#33734D");//#9600B0  changed
  private  Color kOrangeTarget = new Color("#6D7717");//#FF7B00  changed
  private  Color kEmptyTarget = new Color("#4D733B");//#4D733B   new
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
    } else if (this.getClosestColor().color == kGreenTarget ) { //&& this.getClosestColor().confidence > 0.75 && this.getClosestColor().confidence < 1
      colorString = "Green";
    } else if (this.getClosestColor().color == kYellowTarget) { //|| (this.getClosestColor().color == kGreenTarget && this.getClosestColor().confidence > 0.5 && this.getClosestColor().confidence < 0.75)
      colorString = "Yellow";
    }else if (this.getClosestColor().color == kPurpleTarget && this.getClosestColor().confidence > 0.95) {
      colorString = "Purple";
    }else if (this.getClosestColor().color == kOrangeTarget) {
      colorString = "Orange";
    }else if(this.getClosestColor().color == kEmptyTarget){
      colorString = "Empty";
    }else {
      colorString = "Unknown";
    }

    // if(tickingForAvg==0){
    //   averageR = this.getClosestColor().color.red;
    //   averageG = this.getClosestColor().color.green;
    //   averageB = this.getClosestColor().color.blue;

    //   m_colorMatcher.addColorMatch(kBlueTarget);
    //   m_colorMatcher.addColorMatch(kGreenTarget);
    //   m_colorMatcher.addColorMatch(kRedTarget);
    //   m_colorMatcher.addColorMatch(kYellowTarget);   
    //   m_colorMatcher.addColorMatch(kPurpleTarget);
    //   m_colorMatcher.addColorMatch(kOrangeTarget);
    // }else if(tickingForAvg==1){
    //   averageR=(averageR+this.getClosestColor().color.red)/2;
    //   averageG=(averageG+this.getClosestColor().color.green)/2;
    //   averageB=(averageB+this.getClosestColor().color.blue)/2;
    // }else if(tickingForAvg==3){
    //   tickingForAvg=1;
    // }
    // tickingForAvg++;

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

    SmartDashboard.putNumber("Red", this.getDetectedColor().red);
    SmartDashboard.putNumber("Green", this.getDetectedColor().green);
    SmartDashboard.putNumber("Blue", this.getDetectedColor().blue);

    SmartDashboard.putNumber("Confidence Num", this.getClosestColor().confidence);
    SmartDashboard.putString("Confidence", confidence);

    SmartDashboard.putString("Detected Color", colorString);

    // SmartDashboard.putNumber("AvgR", averageR);    
    // SmartDashboard.putNumber("AvgG", averageG);
    // SmartDashboard.putNumber("AvgB", averageB);
  }
}
