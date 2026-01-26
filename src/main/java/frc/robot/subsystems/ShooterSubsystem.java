import com.revrobotics.spark.SparkMax;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.ResetMode;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.AprilTags.AprilTag10;
import frc.robot.Constants.AprilTags.AprilTag2;
import frc.robot.Constants.AprilTags.AprilTag5;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.Constants.AprilTags;

import edu.wpi.first.units.measure.Angle;

import frc.robot.LimelightHelpers;

public class ShooterSubsystem extends SubsystemBase {
    SparkMax flyWheelMotor;
    SparkMax turningMotor;
    SparkMaxConfig flyWheelMotorConfig;
    SparkMaxConfig turningMotorConfig;

    RelativeEncoder flyWheelEncoder;
    AbsoluteEncoder turningEncoder;

    PIDController flyWheelPID;

    private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);
    public ShooterSubsystem() {
        flyWheelMotor = new SparkMax(ShooterConstants.flyWheelMotorid, MotorType.kBrushless);
        turningMotor = new SparkMax(ShooterConstants.turningMotorid, MotorType.kBrushless);
        flyWheelMotorConfig = new SparkMaxConfig(); 
        turningMotorConfig = new SparkMaxConfig();

        flyWheelMotorConfig
        .inverted(false)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(50);
        turningMotorConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(50);

        flyWheelMotor.configure(flyWheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turningMotor.configure(turningMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        flyWheelEncoder = flyWheelMotor.getEncoder();
        turningEncoder = turningMotor.getAbsoluteEncoder();

        flyWheelPID = new PIDController(0, 0, 0);
    }

    public double yDistanceToFidicual(double tync, double fidicualHeight) {
        double h = fidicualHeight - ShooterConstants.limelightHeight;
        double angle = Math.tan(tync + ShooterConstants.limelightMountAngle.in(Radians));
        return h/angle;   
    }

    public double xDistanceToFidicual(double txnc, double dy, double gyroAngle) {
        return dy * Math.tan(gyroAngle - txnc);
    }

    public double xCameraShooterOffset(double gyroAngle) {
        return ShooterConstants.limelightDistanceCenter*Math.sin(gyroAngle) - ShooterConstants.shooterDistanceCenter*Math.sin(gyroAngle+ShooterConstants.shooterAngleOffset.in(Radians));
    }

    public double yCameraShooterOffset(double gyroAngle) {
        return ShooterConstants.limelightDistanceCenter*Math.cos(gyroAngle) - ShooterConstants.shooterDistanceCenter*Math.cos(gyroAngle+ShooterConstants.shooterAngleOffset.in(Radians));
    }

    public double shooterAngleToTarget() {
        try {         
            RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(ShooterConstants.limelightName);

            double o_x;
            double o_y;
            double d_y;
            double d_x;

            double aprilTagHeight;
            double aprilTagXDis;
            double aprilTagYDis;

            for (RawFiducial fiducial : fiducials){
                switch (fiducial.id) {
                    case 2:
                        aprilTagHeight = AprilTag2.height;
                        aprilTagXDis = AprilTag2.xDis;
                        aprilTagYDis = 0;
                        break;
                    case 5:
                        aprilTagHeight = AprilTag5.height;
                        aprilTagXDis = AprilTag5.xDis;
                        aprilTagYDis = 0;
                        break;
                    case 10:
                        aprilTagHeight = AprilTag10.height;
                        aprilTagXDis = 0;
                        aprilTagYDis = AprilTag10.yDis;
                        break;
                    default:
                        break;
                }
                if (fiducial.id == 2 || fiducial.id == 5 || fiducial.id == 10){
                    o_x = xCameraShooterOffset(getAngle().in(Radians));
                    o_y = yCameraShooterOffset(getAngle().in(Radians));

                    d_y = yDistanceToFidicual(fiducial.tync, aprilTagHeight);
                    d_x = xDistanceToFidicual(fiducial.txnc, d_y, getAngle().in(Radians));

                    return Math.atan2(d_x + o_x + aprilTagXDis, d_y + o_y + aprilTagYDis);
                }
            }

        } catch (Exception e){
            System.out.println("LimelightError: ShooterSub Line: 59");
        }
        return 0;
    }

    public Angle getAngle() {
        Angle angle = Degrees.of(m_gyro.getAngle());
        return angle;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turning Anlge", turningEncoder.getPosition());
        SmartDashboard.putNumber("FlyWheel RPM", flyWheelEncoder.getVelocity());
    }

    public void runFlyWheel(double speed) {
        flyWheelMotor.set(speed);
    }

    public void setRPM(double RPM) {
        flyWheelPID.setSetpoint(RPM);
    }

    public void runFlyWheel() {
        double output = flyWheelPID.calculate(flyWheelEncoder.getVelocity());
        output = MathUtil.clamp(output, 0, 1);
        runFlyWheel(output);
    }
}
