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

public class IntakeWrist extends SubsystemBase implements IPositionControlledSubsystem {

	private boolean isHoldingPosition = false;

    // Set Different Heights
	private int homePosition = 0;
	private int maxUpTravelPosition = 50000;

	private int humanPlayerPosition = 23000;
	private int subwooferShotPosition = 33000;
	private int podiumShotPosition = 36000;
	private int frontAmpShotPosition = 39000;
	private int rearAmpShotPosition = 42000;

	private int highScorePosition = 15000;

	public int upPositionLimit = maxUpTravelPosition;
	public int downPositionLimit = 0;
	private int targetPosition = 0;
    private MotionMagicDutyCycle targetPositionDutyCycle = new MotionMagicDutyCycle(0);
	private double feedForward = 0.011;

	private final static int onTargetThreshold = 2000;
		
	private TalonFX intakeWristFalcon = new TalonFX(17);
	private TalonFX intakeWristFalconFollower = new TalonFX(18);

    private TalonFXConfiguration intakeWristFXConfig = new TalonFXConfiguration();

	public IntakeWrist() {
		// Clear Sticky Faults
		this.intakeWristFalcon.clearStickyFaults();
		this.intakeWristFalconFollower.clearStickyFaults();
		
        // Set Followers
		this.intakeWristFalconFollower.setControl(new Follower(17, true));

        /** intake Motor Configuration */
        /* Motor Inverts and Neutral Mode */
		intakeWristFXConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        intakeWristFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        /* Current Limiting */
        intakeWristFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeWristFXConfig.CurrentLimits.SupplyCurrentLimit = 35;
        intakeWristFXConfig.CurrentLimits.SupplyCurrentThreshold = 60;
        intakeWristFXConfig.CurrentLimits.SupplyTimeThreshold = 0.1;

        /* PID Config */
        intakeWristFXConfig.Slot0.kP = 0.8;
        intakeWristFXConfig.Slot0.kI = 0;
        intakeWristFXConfig.Slot0.kD = 0;

        /* Open and Closed Loop Ramping */
        intakeWristFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25;
        intakeWristFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25;

        intakeWristFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.25;
        intakeWristFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.25;

        //Config Acceleration and Velocity
        intakeWristFXConfig.MotionMagic.withMotionMagicAcceleration(24000);
        intakeWristFXConfig.MotionMagic.withMotionMagicCruiseVelocity(20000);

        // Config Motor
        intakeWristFalcon.getConfigurator().apply(intakeWristFXConfig);
        intakeWristFalcon.getConfigurator().setPosition(0.0);
		intakeWristFalconFollower.getConfigurator().setPosition(0);
	}

	public void motionMagicControl() {
		this.manageMotion(targetPosition);
        targetPositionDutyCycle.withPosition(targetPosition);
        targetPositionDutyCycle.withFeedForward(feedForward);
		this.intakeWristFalcon.setControl(targetPositionDutyCycle);
	}

	public double getCurrentPosition() {
		return this.intakeWristFalcon.getRotorPosition().getValueAsDouble();
	}

	public double getCurrentDraw() {
		return this.intakeWristFalcon.getSupplyCurrent().getValueAsDouble();
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

	public boolean setTargetPosition(int position) {
		if (!isValidPosition(position)) {
			return false;
		} else {
			this.targetPosition = position;
			return true;
		}
	}

	public void forceSetTargetPosition(int position) {
		this.targetPosition = position;
	}

	public void incrementTargetPosition(int increment) {
		int currentTargetPosition = this.targetPosition;
		int newTargetPosition = currentTargetPosition + increment;
		if (isValidPosition(newTargetPosition)) {		// && isWristSafe(newTargetPosition) check for other subsystems
			this.targetPosition = newTargetPosition;
		}
	}

	public boolean isValidPosition(int position) {
		boolean withinBounds = position <= upPositionLimit && position >= downPositionLimit;
		return withinBounds;
	}

    // communicate with commands
	public int getHomePosition() {
		return this.homePosition;
	}

	public int getMaxUpTravelPosition() {
		return this.maxUpTravelPosition;
	}


	public int getHumanPlayerPosition() {
		return this.humanPlayerPosition;
	}

	public int getPodiumShotPosition() {
		return this.podiumShotPosition;
	}

	public int getSubwooferShotPosition() {
		return this.subwooferShotPosition;
	}

	public int getFrontAmpShotPosition() {
		return this.frontAmpShotPosition;
	}

	public int getRearAmpShotPosiiton() {
		return this.rearAmpShotPosition;
	}

	public int getHighScorePosition() {
		return this.highScorePosition;
	}

	public double getFeedForward() {
		return this.feedForward;
	}

	public void resetWristEncoder() {
        try {
			intakeWristFalcon.getConfigurator().setPosition(0.0);
			intakeWristFalconFollower.getConfigurator().setPosition(0);
        }
        catch (Exception e) {
            DriverStation.reportError("Wrist.resetWristEncoders exception.  You're Screwed! : " + e.toString(), false);
        }
	}

	public double JoyStickIntakeWrist(){
		double value = 0;
		value = Robot.robotContainer.getOperatorLeftStickY();
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
		SmartDashboard.putNumber("Intake Wrist Position", this.getCurrentPosition());
		SmartDashboard.putNumber("Intake Wrist Target Position", this.getTargetPosition());
		SmartDashboard.putNumber("Intake Wrist Position Error", this.getPositionError());
		SmartDashboard.putNumber("Intake Wrist Velocity", this.getCurrentVelocity());
		SmartDashboard.putNumber("Intake Wrist Current", this.getCurrentDraw());
	}

	@Override
	public double getCurrentVelocity() {
		double currentVelocity = this.intakeWristFalcon.getVelocity().getValueAsDouble();
		return currentVelocity;
	}

	@Override
	public boolean isInPosition(int targetPosition) {
		double currentPosition = this.getCurrentPosition();
		double positionError = Math.abs(currentPosition - targetPosition);
		if (positionError < onTargetThreshold) {
			return true;
		} else {
			return false;
		}
	}
}   
