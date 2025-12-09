package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import static frc.robot.Constants.MotorConstants.k_woodSpeed;

public class WoodSubsystem extends SubsystemBase
{
    private Talon woodMotor;

    public WoodSubsystem() {
        woodMotor = new Talon(MotorConstants.k_woodID);
    }

    public void runWood(double polarity) {
        woodMotor.set(polarity*k_woodSpeed);
    }

    public void stopWood() {
        woodMotor.set(0);
    }

    @Override
    public void periodic() {}

    @Override
    public void simulationPeriodic() {}
}