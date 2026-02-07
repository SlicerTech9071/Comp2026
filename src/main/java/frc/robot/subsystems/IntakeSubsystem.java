package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.intakeConstants;

public class IntakeSubsystem extends SubsystemBase{
    SparkMax intakeMotor;
    SparkMax pivotMotor;

    SparkMaxConfig intakeMotorConfig;
    SparkMaxConfig pivotMotorConfig;
    PIDController pivotMotorPID;
    RelativeEncoder pivotMotorEncoder;
    public IntakeSubsystem() {
        intakeMotor = new SparkMax(intakeConstants.intakeMotorid, MotorType.kBrushless);
        pivotMotor = new SparkMax(intakeConstants.pivotMotorid, MotorType.kBrushless);
        pivotMotorEncoder = pivotMotor.getEncoder();

        intakeMotorConfig = new SparkMaxConfig();
        pivotMotorConfig = new SparkMaxConfig();
        pivotMotorPID = new PIDController(0, 0, 0);
    }

    @Override
    public void periodic(){

    }

    public void setIntakePos(double setPoint){
        pivotMotorPID.setSetpoint(setPoint);
    }
    public void moveIntaketoPos (){
        double Speed = pivotMotorPID.calculate(pivotMotorEncoder.getPosition());
        pivotMotor.set(Speed);
    }
}
