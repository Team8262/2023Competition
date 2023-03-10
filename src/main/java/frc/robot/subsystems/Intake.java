package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.Constants.*;

public class Intake extends SubsystemBase{
    private CANSparkMax intakeMotor;

    int motorID = Constants.intakeMotor;
    
    public Intake(){
        intakeMotor = new CANSparkMax(motorID, MotorType.kBrushless);
    }

    @Override
    public void periodic(){}

    public void setSpeed(double speed){
        intakeMotor.set(speed);
    }
}