package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.hopperConstants;

//HopperSub control any storing and indexing of any fuel
public class HopperSubsystem extends SubsystemBase{
     SparkMax hopperIndexerMotorTop;
     SparkMax hopperIndexerMotorBottom;
     
     SparkMax expansionMotorLeft;
     SparkMax expansionMotorRight;
     
     RelativeEncoder expansionEncoder;

     SparkMaxConfig hopperIndexerConfigTop;
     SparkMaxConfig hopperIndexerConfigBottom;
     SparkMaxConfig expansionConfigLeft;
     SparkMaxConfig expansionConfigRight;
    

     public HopperSubsystem() {
        hopperIndexerMotorTop = new SparkMax(hopperConstants.indexerMotorTopid, MotorType.kBrushless);
        hopperIndexerMotorBottom = new SparkMax(hopperConstants.indexerMotorBottomid, MotorType.kBrushless);
        expansionMotorLeft = new SparkMax(hopperConstants.expansionMotorLeftid, MotorType.kBrushless);
        expansionMotorRight = new SparkMax(hopperConstants.expansionMotorRightid, MotorType.kBrushless);

        hopperIndexerConfigTop = new SparkMaxConfig();
        hopperIndexerConfigBottom = new SparkMaxConfig();
        expansionConfigLeft = new SparkMaxConfig();
        expansionConfigRight = new SparkMaxConfig();

        hopperIndexerConfigTop
        .idleMode(IdleMode.kCoast);
        hopperIndexerConfigBottom
        .idleMode(IdleMode.kCoast);

        expansionConfigLeft
        .idleMode(IdleMode.kBrake);
        expansionConfigRight
        .idleMode(IdleMode.kBrake);
    
        expansionEncoder = expansionMotorLeft.getEncoder();
        
        hopperIndexerMotorTop.configure(hopperIndexerConfigTop, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        hopperIndexerMotorBottom.configure(hopperIndexerConfigBottom, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        expansionMotorLeft.configure(expansionConfigLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        expansionMotorRight.configure(expansionConfigRight, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);

    }

    @Override
    public void periodic() {

    }

    public void runExpansionMotors(int foward) {
        expansionMotorLeft.set(-1*hopperConstants.hopperSpeed * foward);
        expansionMotorRight.set(hopperConstants.hopperSpeed * foward);
    }

    public void runIndexer() {
        hopperIndexerMotorTop.set(hopperConstants.indexerSpeed);
        hopperIndexerMotorBottom.set(-1*hopperConstants.indexerSpeed);
    }

}
