package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Intake extends SubsystemBase{
    private CANSparkMax intakeMotor;
    private CANSparkMax sideMotor;

    int motorID = Constants.intakeMotor;
    
    public Intake(){
        intakeMotor = new CANSparkMax(motorID, MotorType.kBrushless);
        sideMotor = new CANSparkMax(Constants.sideIntakeMotor, MotorType.kBrushless);
    }

    @Override
    public void periodic(){}

    public void setSpeed(double speed){
        intakeMotor.set(speed);
        sideMotor.set(speed);
    }
}