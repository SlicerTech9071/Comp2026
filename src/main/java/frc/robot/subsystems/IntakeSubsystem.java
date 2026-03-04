package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.intakeConstants;

//IntakeSub controls anything that picks up fuel
public class IntakeSubsystem extends SubsystemBase{
    SparkMax intakeMotor;

    SparkMaxConfig intakeMotorConfig;
    public IntakeSubsystem() {
        intakeMotor = new SparkMax(intakeConstants.intakeMotorid, MotorType.kBrushless);

        intakeMotorConfig = new SparkMaxConfig();

    }

    @Override
    public void periodic(){

    }

        public void runIntake(double speed) {
            intakeMotor.set(speed);
        }
}
