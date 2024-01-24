package frc.robot.subsystems;

import frc.robot.Robot;
import frc.lib.models.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RightShooter extends SubsystemBase implements IVelocityControlledSubsystem {

	private boolean isHoldingVelocity = false;

	// Set Different Speeds
	private double conversionFactor = 4096 / 600;
	private double zeroVelocity = 0*conversionFactor;
	private double maxVelocity = 5000*conversionFactor;

	public final static int Shooter_PIDX = 0;

	public double maxVelocityLimit = maxVelocity;
	public double lowVelocityLimit = 0;
	private VelocityDutyCycle targetVelocityDutyCycle = new VelocityDutyCycle(0);

    public double targetVelocity = 0;
	private double arbitraryFeedForward = 0.0;

	private final static double onTargetThreshold = 1;

	public TalonFX RightShooterFalcon = new TalonFX(14, "canivore");
    public TalonFXConfiguration RightShooterFXConfig = new TalonFXConfiguration();

	public RightShooter() {
        /** Shooter Motor Configuration */
        /* Motor Inverts and Neutral Mode */
		RightShooterFXConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        RightShooterFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        /* Current Limiting */
        RightShooterFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        RightShooterFXConfig.CurrentLimits.SupplyCurrentLimit = 35;
        RightShooterFXConfig.CurrentLimits.SupplyCurrentThreshold = 60;
        RightShooterFXConfig.CurrentLimits.SupplyTimeThreshold = 0.1;

        /* PID Config */
        RightShooterFXConfig.Slot0.kP = 0.1;
        RightShooterFXConfig.Slot0.kI = 0;
        RightShooterFXConfig.Slot0.kD = 0;

        /* Open and Closed Loop Ramping */
        RightShooterFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 1;
        RightShooterFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 1;

        RightShooterFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 1;
        RightShooterFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 1;

        //Config Acceleration and Velocity
        RightShooterFXConfig.MotionMagic.withMotionMagicAcceleration(10000);
        RightShooterFXConfig.MotionMagic.withMotionMagicCruiseVelocity(20000);

        // Config Motor
        RightShooterFalcon.getConfigurator().apply(RightShooterFXConfig);
        RightShooterFalcon.getConfigurator().setPosition(0.0);

	}

	public void velocityControl() {
		targetVelocityDutyCycle.withVelocity(targetVelocity);
		this.RightShooterFalcon.setControl(targetVelocityDutyCycle);
	}

	public double getCurrentDraw() {
		return this.RightShooterFalcon.getSupplyCurrent().getValueAsDouble();
	}

	public boolean isHoldingVelocity() {
		return this.isHoldingVelocity;
	}

	public void setIsHoldingVelocity(boolean isHoldingVelocity) {
		this.isHoldingVelocity = isHoldingVelocity;
	}

	public double getTargetVelocity() {
		return this.targetVelocity;
	}

	public boolean setTargetVelocity(double Velocity) {
		if (!isValidVelocity(Velocity)) {
			return false;
		} else {
			this.targetVelocity = Velocity;
			return true;
		}
	}

	public void forceSetTargetVelocity(double Velocity) {
		this.targetVelocity = Velocity;
	}

	public void incrementTargetVelocity(double increment) {
		double currentTargetVelocity = this.targetVelocity;
		double newTargetVelocity = currentTargetVelocity + increment;
		if (isValidVelocity(newTargetVelocity)) {
			this.targetVelocity = newTargetVelocity;
		}
	}

	public boolean isValidVelocity(double Velocity) {
		boolean withinBounds = Velocity <= maxVelocityLimit && Velocity >= lowVelocityLimit;
		return withinBounds;
	}

    // communicate with commands
	public double getZeroVelocity() {
		return this.zeroVelocity;
	}

	public double getMaxVelocity() {
		return this.maxVelocity;
	}

	public double getArbitraryFeedForward() {
		return this.arbitraryFeedForward;
	}

	public void resetShooterEncoder() {
        try {
			RightShooterFalcon.getConfigurator().setPosition(0.0);
        }
        catch (Exception e) {
            DriverStation.reportError("Shooter.resetShooterEncoders exception.  You're Screwed! : " + e.toString(), false);
        }
	}

	public double getVelocityError() {
		double currentVelocity = this.getCurrentVelocity();
		double targetVelocity = this.getTargetVelocity();
		double VelocityError = Math.abs(currentVelocity - targetVelocity);
		return VelocityError;
	}

	public double joystickShooter(){
		double value = 0;
		value = -Robot.robotContainer.getOperatorRightStickY();
		return value;
	}

	public void updateDashboard() {
		SmartDashboard.putNumber("Right Shooter Velocity", this.getCurrentVelocity());
		SmartDashboard.putNumber("Right Shooter Target Velocity", this.getTargetVelocity());
		SmartDashboard.putNumber("Right Shooter Velocity Error", this.getVelocityError());
		SmartDashboard.putNumber("Right Shooter Current", this.getCurrentDraw());
	}

	@Override
	public double getCurrentVelocity() {
		double currentVelocity = this.RightShooterFalcon.getVelocity().getValueAsDouble();
		return currentVelocity;
	}

	@Override
	public boolean isAtVelocity(double targetVelocity) {
		double currentVelocity = this.getCurrentVelocity();
		double VelocityError = Math.abs(currentVelocity - targetVelocity);
		if (VelocityError < onTargetThreshold) {
			return true;
		} else {
			return false;
		}
	}
}