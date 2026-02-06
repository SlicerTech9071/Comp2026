package frc.robot.subsystems;

import java.lang.management.MemoryType;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.intakeConstants;

public class IntakeSubsystem extends SubsystemBase{
    public SparkMax intakeMotor;
    public SparkMax pivotMotor;

    public SparkMaxConfig intakeMotorConfig;
    public SparkMaxConfig pivotMotorConfig;

    public IntakeSubsystem() {
        intakeMotor = new SparkMax(intakeConstants.intakeMotorid, MotorType.kBrushless);
        pivotMotor = new SparkMax(intakeConstants.pivotMotorid, MotorType.kBrushless);

        intakeMotorConfig = new SparkMaxConfig();
        pivotMotorConfig = new SparkMaxConfig();

    }

    @Override
    public void periodic(){

    }
}
