package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.HopperConstants;

public class HopperSubystem extends SubsystemBase{
    public Spark hopperLinearActuator;
    public SparkMax hopperMotor;
   
    public DigitalInput hopperLimit;

    public SparkMaxConfig hopperMotorConfig;

    public HopperSubystem() {
        hopperLinearActuator = new Spark(HopperConstants.hopperLinearActuatorid);
        hopperMotor = new SparkMax(HopperConstants.hopperMotorid, MotorType.kBrushless);

        hopperLimit = new DigitalInput(HopperConstants.hopperLimitid);

        hopperMotorConfig = new SparkMaxConfig();

        hopperMotor.configure(hopperMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic(){

    }

    public void runHopper(double speed){
        hopperMotor.set(speed);
    }

    public void extendHopper(){
        if (hopperLimit.get() == true){
            hopperLinearActuator.set(HopperConstants.hopperExtensionSpeed);
        }
    }


}
