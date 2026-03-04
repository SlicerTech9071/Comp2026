package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

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
     SparkMaxConfig hopperIndexerrConfigBottom;
     SparkMaxConfig expansionConfigLeft;
     SparkMaxConfig expansionConfigRight;
    

     public HopperSubsystem() {
        hopperIndexerMotorTop = new SparkMax(hopperConstants.indexerMotorTopid, MotorType.kBrushless);
        hopperIndexerMotorBottom = new SparkMax(hopperConstants.indexerMotorBottomid, MotorType.kBrushless);
        expansionMotorLeft = new SparkMax(hopperConstants.expansionMotorLeftid, MotorType.kBrushless);
        expansionMotorRight = new SparkMax(hopperConstants.expansionMotorRightid, MotorType.kBrushless);

        hopperIndexerConfigTop = new SparkMaxConfig();
        hopperIndexerrConfigBottom = new SparkMaxConfig();
        expansionConfigLeft = new SparkMaxConfig();
        expansionConfigRight = new SparkMaxConfig();
    
        expansionEncoder = expansionMotorLeft.getEncoder();
        
        hopperIndexerMotorTop.configure(hopperIndexerConfigTop, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        hopperIndexerMotorBottom.configure(hopperIndexerrConfigBottom, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        expansionMotorLeft.configure(expansionConfigLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        expansionMotorRight.configure(expansionConfigRight, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);

    }

    @Override
    public void periodic() {

    }

    public void runExpansionMotors() {
        expansionMotorLeft.set(-1*hopperConstants.hopperSpeed);
        expansionMotorRight.set(hopperConstants.hopperSpeed);
    }

    public void runIndexer() {
        hopperIndexerMotorTop.set(hopperConstants.indexerSpeed);
        hopperIndexerMotorBottom.set(-1*hopperConstants.indexerSpeed);
    }

}
