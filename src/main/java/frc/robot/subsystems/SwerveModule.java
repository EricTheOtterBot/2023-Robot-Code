package frc.robot.subsystems;

//import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
//import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import com.revrobotics.RelativeEncoder;
//import com.revrobotics.AbsoluteEncoder;
//import com.revrobotics.SparkMaxAbsoluteEncoder;
//import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.kinematics.SwerveModulePosition;



public class SwerveModule{

    private CANSparkMax drive_motor;
    private CANSparkMax turn_motor;
    private RelativeEncoder drive_encoder;
    private RelativeEncoder turn_encoder;
    private CANCoder absolute_encoder;
    private PIDController turningPidController;
    private boolean absoluteEncoderReversed;
    private double absoluteEncoderOffsetRad;


    public SwerveModule(int driveMotorId, int turnMotorId, boolean driveMotorReversed, 
                        boolean turningMotorReversed, int absoluteEncoderId, 
                        double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        drive_motor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turn_motor = new CANSparkMax(turnMotorId, MotorType.kBrushless);

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absolute_encoder = new CANCoder(absoluteEncoderId);

        drive_motor.setInverted(driveMotorReversed);
        turn_motor.setInverted(turningMotorReversed);

        drive_encoder = drive_motor.getEncoder();
        turn_encoder = turn_motor.getEncoder();

        drive_encoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        drive_encoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turn_encoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turn_encoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }


    public double getDriveVelocity() {
        return drive_encoder.getVelocity();
    }


    public SwerveModuleState getState() {
        return new SwerveModuleState(
            drive_encoder.getVelocity(), new Rotation2d(turn_encoder.getPosition()));
      }
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(drive_encoder.getPosition(), new Rotation2d(turn_encoder.getPosition()));
    }
    public double getAbsoluteEncoderRad() {
        double angle = absolute_encoder.getPosition();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }
    public void resetEncoders() {
        drive_encoder.setPosition(0);
        turn_encoder.setPosition(getAbsoluteEncoderRad());
    }
    public void stop() {
        drive_motor.set(0);
        turn_motor.set(0);
    }
    
    public double getTurningPosition() {
        return turn_encoder.getPosition();
    }


    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        drive_motor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turn_motor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        
    }
}
