package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Drivetrain extends SubsystemBase {

    private final CANSparkMax leftLead = new CANSparkMax(1, MotorType.kBrushless);
    private final CANSparkMax leftFollower = new CANSparkMax(2, MotorType.kBrushless);
    private final CANSparkMax rightLead = new CANSparkMax(3, MotorType.kBrushless);
    private final CANSparkMax rightFollower = new CANSparkMax(4, MotorType.kBrushless);

    private final RelativeEncoder leftEncoder = leftLead.getEncoder();
    private final RelativeEncoder rightEncoder = rightLead.getEncoder();

    private Pigeon2 pigeon = new Pigeon2(12);
  
    public Drivetrain() {

        rightLead.clearFaults();
        rightLead.restoreFactoryDefaults();
        leftLead.clearFaults();
        leftLead.restoreFactoryDefaults();
        rightFollower.clearFaults();
        rightFollower.restoreFactoryDefaults();
        leftFollower.clearFaults();
        leftFollower.restoreFactoryDefaults();

        // Setup Followers
        leftFollower.follow(leftLead);
        rightFollower.follow(rightLead);

        leftLead.setInverted(true);
        rightLead.setInverted(false);
        leftFollower.setInverted(true);
        rightFollower.setInverted(false);
        
        // Zero Sensors
        resetPigeon();
        resetDriveEncoders();
        leftLead.setIdleMode(IdleMode.kBrake);
        leftFollower.setIdleMode(IdleMode.kBrake);
        rightLead.setIdleMode(IdleMode.kBrake);
        rightFollower.setIdleMode(IdleMode.kBrake);


        // Set Ramp Rates
        leftLead.setOpenLoopRampRate(0.25);
        rightLead.setOpenLoopRampRate(0.25);

        leftLead.setClosedLoopRampRate(0.25);
        rightLead.setClosedLoopRampRate(0.25);
    }

    public void teleopDrive() {
        double moveValue = 0;
        double rotateValue = 0;

        double left = 0;
        double right = 0;

        moveValue = Robot.robotContainer.getDriverLeftStickY();
        rotateValue = -Robot.robotContainer.getDriverRightStickX();

        left = moveValue + rotateValue;
        right = moveValue - rotateValue;

        leftLead.set(left);
        rightLead.set(right);
    }
    
    public double getAngle() {
        double[] ypr = new double[3];
        pigeon.getYawPitchRoll(ypr);
        return ypr[0];
    }
    
    public double getRoll() {
        double[] ypr = new double[3];
        pigeon.getYawPitchRoll(ypr);
        return ypr[2];
    }

    public void resetPigeon() {
        this.pigeon.setYaw(0.0, 0);
    }

    public void resetDriveEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    public double getRightDistance() {
        return rightEncoder.getPosition();
    }

    public double getLeftDistance() {
        return leftEncoder.getPosition();
    }

    public double getLeftSpeed() {
        return leftEncoder.getVelocity();
    }

    public double getRightSpeed() {
        return rightEncoder.getVelocity();
    }

    public void updateDashboard() {
        SmartDashboard.putNumber("Drivetrain / Left Lead Current", leftLead.getOutputCurrent());
        SmartDashboard.putNumber("Drivetrain / Right Lead Current", rightLead.getOutputCurrent());
        SmartDashboard.putNumber("Drivetrain / Left Speed", getLeftSpeed());
        SmartDashboard.putNumber("Drivetrain / Right Speed", getRightSpeed());
        SmartDashboard.putNumber("Drivetrain / Current Angle", getAngle());
        SmartDashboard.putNumber("Drivetrain / Current Angular Rate", getRoll());
        SmartDashboard.putNumber("Drivetrain / Left Distance", getLeftDistance());
        SmartDashboard.putNumber("Drivetrain / Right Distance", getRightDistance());
    }
}
