package frc.robot.subsystems;

import frc.lib.models.*;
import frc.robot.Robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterWrist extends SubsystemBase implements IPositionControlledSubsystem {

	private boolean isHoldingPosition = false;

    // Set Different Heights
	private double homePosition = 0;
	private double maxUpTravelPosition = 105;

	private double humanPlayerPosition = 100;
	private double subwooferShotPosition = 90;
	private double podiumShotPosition = 80;
	private double frontAmpShotPosition = 70;
	private double rearAmpShotPosition = 30;

	private double highScorePosition = 20;

	public double upPositionLimit = maxUpTravelPosition;
	public double downPositionLimit = 0;
	private double targetPosition = 0;
    private MotionMagicDutyCycle targetPositionDutyCycle = new MotionMagicDutyCycle(0);
	private double feedForward = 0.0;

	private final static double onTargetThreshold = 1;
		
	private TalonFX shooterWristFalcon = new TalonFX(15, "canivore");
	private TalonFX shooterWristFalconFollower = new TalonFX(16, "canivore");

    private TalonFXConfiguration shooterWristFXConfig = new TalonFXConfiguration();

	public ShooterWrist() {
		// Clear Sticky Faults
		this.shooterWristFalcon.clearStickyFaults();
		this.shooterWristFalconFollower.clearStickyFaults();
		
        // Set Followers
		this.shooterWristFalconFollower.setControl(new Follower(15, true));

        /** Shooter Motor Configuration */
        /* Motor Inverts and Neutral Mode */
		shooterWristFXConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        shooterWristFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        /* Current Limiting */
        shooterWristFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooterWristFXConfig.CurrentLimits.SupplyCurrentLimit = 35;
        shooterWristFXConfig.CurrentLimits.SupplyCurrentThreshold = 60;
        shooterWristFXConfig.CurrentLimits.SupplyTimeThreshold = 0.1;

        /* PID Config */
        shooterWristFXConfig.Slot0.kP = 0.05;
        shooterWristFXConfig.Slot0.kI = 0;
        shooterWristFXConfig.Slot0.kD = 0;

        /* Open and Closed Loop Ramping */
        shooterWristFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25;
        shooterWristFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25;

        shooterWristFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.25;
        shooterWristFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.25;

        //Config Acceleration and Velocity
        shooterWristFXConfig.MotionMagic.withMotionMagicAcceleration(100);
        shooterWristFXConfig.MotionMagic.withMotionMagicCruiseVelocity(100);

        // Config Motor
        shooterWristFalcon.getConfigurator().apply(shooterWristFXConfig);
        shooterWristFalcon.getConfigurator().setPosition(0.0);
		shooterWristFalconFollower.getConfigurator().setPosition(0);
	}

	public void motionMagicControl() {
		this.manageMotion(targetPosition);
        targetPositionDutyCycle.withPosition(targetPosition);
        targetPositionDutyCycle.withFeedForward(feedForward);
		this.shooterWristFalcon.setControl(targetPositionDutyCycle);
	}

	public double getCurrentPosition() {
		return this.shooterWristFalcon.getRotorPosition().getValueAsDouble();
	}

	public double getCurrentDraw() {
		return this.shooterWristFalcon.getSupplyCurrent().getValueAsDouble();
	}

	public boolean isHoldingPosition() {
		return this.isHoldingPosition;
	}

	public void setIsHoldingPosition(boolean isHoldingPosition) {
		this.isHoldingPosition = isHoldingPosition;
	}

	public double getTargetPosition() {
		return this.targetPosition;
	}

	public boolean setTargetPosition(double position) {
		if (!isValidPosition(position)) {
			return false;
		} else {
			this.targetPosition = position;
			return true;
		}
	}

	public void forceSetTargetPosition(double position) {
		this.targetPosition = position;
	}

	public void incrementTargetPosition(double increment) {
		double currentTargetPosition = this.targetPosition;
		double newTargetPosition = currentTargetPosition + increment;
		if (isValidPosition(newTargetPosition)) {		// && isWristSafe(newTargetPosition) check for other subsystems
			this.targetPosition = newTargetPosition;
		}
	}

	public boolean isValidPosition(double position) {
		boolean withinBounds = position <= upPositionLimit && position >= downPositionLimit;
		return withinBounds;
	}

    // communicate with commands
	public double getHomePosition() {
		return this.homePosition;
	}

	public double getMaxUpTravelPosition() {
		return this.maxUpTravelPosition;
	}


	public double getHumanPlayerPosition() {
		return this.humanPlayerPosition;
	}

	public double getPodiumShotPosition() {
		return this.podiumShotPosition;
	}

	public double getSubwooferShotPosition() {
		return this.subwooferShotPosition;
	}

	public double getFrontAmpShotPosition() {
		return this.frontAmpShotPosition;
	}

	public double getRearAmpShotPosiiton() {
		return this.rearAmpShotPosition;
	}

	public double getHighScorePosition() {
		return this.highScorePosition;
	}

	public double getFeedForward() {
		return this.feedForward;
	}

	public void resetWristEncoder() {
        try {
			shooterWristFalcon.getConfigurator().setPosition(0.0);
			shooterWristFalconFollower.getConfigurator().setPosition(0);
        }
        catch (Exception e) {
            DriverStation.reportError("Wrist.resetWristEncoders exception.  You're Screwed! : " + e.toString(), false);
        }
	}

	public double JoystickShooterWrist(){
		double value = 0;
		value = -Robot.robotContainer.getOperatorRightStickY();
		return value;
	}

	public double getPositionError() {
		double currentPosition = this.getCurrentPosition();
		double targetPosition = this.getTargetPosition();
		double positionError = Math.abs(currentPosition - targetPosition);
		return positionError;
	}

	public void manageMotion(double targetPosition) {
		double currentPosition = getCurrentPosition();
		if (currentPosition < targetPosition) {
				// set based on gravity
		}
		else {
				//set based on gravity
		}
	}

	public void zeroTarget() {
		targetPosition = 0;
	}

	public void updateDashboard() {
		SmartDashboard.putNumber("Shooter Wrist Position", this.getCurrentPosition());
		SmartDashboard.putNumber("Shooter Wrist Target Position", this.getTargetPosition());
		SmartDashboard.putNumber("Shooter Wrist Position Error", this.getPositionError());
		SmartDashboard.putNumber("ShooterWrist Velocity", this.getCurrentVelocity());
		SmartDashboard.putNumber("Shooter Wrist Current", this.getCurrentDraw());
	}

	@Override
	public double getCurrentVelocity() {
		double currentVelocity = this.shooterWristFalcon.getVelocity().getValueAsDouble();
		return currentVelocity;
	}

	@Override
	public boolean isInPosition(double targetPosition) {
		double currentPosition = this.getCurrentPosition();
		double positionError = Math.abs(currentPosition - targetPosition);
		if (positionError < onTargetThreshold) {
			return true;
		} else {
			return false;
		}
	}
}   
