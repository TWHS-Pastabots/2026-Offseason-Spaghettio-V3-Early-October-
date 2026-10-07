package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;


import frc.robot.Configs;
import frc.robot.Constants.Ports;

/** Add your docs here. */
public class Intake extends SubsystemBase{
    private SparkMax intakeMotor = new SparkMax(Ports.intake, MotorType.kBrushless);


    public Intake() 
    {
        intakeMotor.configure(Configs.EasySwerveModule.intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    
    public void intake() 
    {
        intakeMotor.set(-0.5);
    }
    
    public void reverseIntake() 
    {
        intakeMotor.set(0.5);
    }

    public void stopIntake() 
    {
        intakeMotor.set(0);
    }
    
    public void setIntake(double speed)
    {
        intakeMotor.set(speed);
    }
}