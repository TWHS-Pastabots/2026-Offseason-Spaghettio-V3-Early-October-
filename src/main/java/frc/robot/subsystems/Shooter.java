package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import frc.robot.Configs;
import frc.robot.Constants.Ports;

public class Shooter {
    private SparkFlex shooterTopMotor = new SparkFlex(Ports.shooterTop, MotorType.kBrushless);
    private SparkFlex shooterFollowerMotor = new SparkFlex(Ports.shooterFollower, MotorType.kBrushless);
    private SparkMax shooterFeederMotor = new SparkMax(Ports.shooterFeeder, MotorType.kBrushless);
    private SparkMax shooterPivotMotor = new SparkMax(Ports.shooterPivot, MotorType.kBrushless);
    
    private ArmFeedforward feedforward = new ArmFeedforward(0,0.5,0);

    private SparkClosedLoopController controller = shooterPivotMotor.getClosedLoopController();

    public Shooter()
    {
        shooterTopMotor.configure(Configs.EasySwerveModule.shooterTopConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterFollowerMotor.configure(Configs.EasySwerveModule.shooterFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}
