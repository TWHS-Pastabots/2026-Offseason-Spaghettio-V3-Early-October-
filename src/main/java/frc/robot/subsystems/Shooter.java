package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ArmFeedforward;
import frc.robot.Configs;
import frc.robot.Constants.Ports;

public class Shooter extends SubsystemBase{
    private SparkMax shooterTopMotor = new SparkMax(Ports.shooterTop, MotorType.kBrushless);
    private SparkMax shooterFollowerMotor = new SparkMax(Ports.shooterFollower, MotorType.kBrushless);
    private SparkMax shooterFeederMotor = new SparkMax(Ports.shooterFeeder, MotorType.kBrushless);
    private SparkMax shooterPivotMotor = new SparkMax(Ports.shooterPivot, MotorType.kBrushless);
    
    private ArmFeedforward feedforward = new ArmFeedforward(0,0.5,0);

    private SparkClosedLoopController controller = shooterPivotMotor.getClosedLoopController();

    public Shooter()
    {
        shooterTopMotor.configure(Configs.EasySwerveModule.shooterTopConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterFollowerMotor.configure(Configs.EasySwerveModule.shooterFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void launch()
    {
        shooterTopMotor.set(0.5);
        shooterFollowerMotor.set(-0.5);
    }

    public void reverseLaunch()
    {
        shooterTopMotor.set(-0.5);
        shooterFollowerMotor.set(0.5);
    }

    public void stopShooter() 
    {
        shooterTopMotor.set(0);
        shooterFollowerMotor.set(0);
    }
    
    public void setShooter(double speed)
    {
        shooterTopMotor.set(speed);
        shooterFollowerMotor.set(speed * -1);
    }

    
}
