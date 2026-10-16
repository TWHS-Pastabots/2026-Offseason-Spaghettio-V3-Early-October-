package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.encoder.SplineEncoder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configs;
import frc.robot.Constants.Ports;

public class Shooter extends SubsystemBase{
    private SparkMax shooterTopMotor = new SparkMax(Ports.shooterTop, MotorType.kBrushless);
    private SparkMax shooterFollowerMotor = new SparkMax(Ports.shooterFollower, MotorType.kBrushless);
    private SparkMax shooterFeederMotor = new SparkMax(Ports.shooterFeeder, MotorType.kBrushless);
    private SparkMax shooterPivotMotor = new SparkMax(Ports.shooterPivot, MotorType.kBrushless);
    
    private ArmFeedforward feedforward = new ArmFeedforward(0,.005,0);

    private SparkClosedLoopController controller = shooterPivotMotor.getClosedLoopController();

    public Shooter()
    {
        shooterTopMotor.configure(Configs.EasySwerveModule.shooterTopConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterFollowerMotor.configure(Configs.EasySwerveModule.shooterFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterPivotMotor.configure(Configs.EasySwerveModule.shooterFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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

    // public double down = 0.15;
	// public double mid = 0.07;
	// public double up = 0.05;
	// public double setpoint = down;

    // private void change()
	// {
	// 	if(setpoint == down)
	// 		setpoint = up;
	// 	else
	// 		setpoint = down;
	// }

    // public void setSetpoint(double point){
    //     setpoint = point;
    // }

    @Override
    public void periodic()
    {
		controller.setSetpoint(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedforward.calculate(Math.toRadians(shooterPivotMotor.getEncoder().getPosition()/2), 0));

		SmartDashboard.putNumber("Pivot Position", shooterPivotMotor.getAbsoluteEncoder().getPosition());
        shooterPivotMotor.set(0.5);
    }


    // /**
	//  * Alternates between the up and down setpoints for the pivot
	//  * 
	//  * @return command to flip the pivot setpoints1
	//  */
    // public Command flip()
	// {
	// 	return new InstantCommand(()->change());
	// }
    
    // /**
	//  * 
	//  * @return command that is used as a way to jostle fuel within the hopper
	//  */
    // public Command mid()
	// {
	// 	return new InstantCommand(()-> {setpoint = setpoint==down?mid:down;}, this);
	// }

    // /**
	//  * raises the low "intaking setpoint"
	//  */
	// public void upMore() 
	// {
	// 	down += 5;
	// }

    // /**
	//  * lowers the low "intaking setpoint"
	//  */
	// public void downMore() 
	// {
	// 	down -= 5;
	// }
}
