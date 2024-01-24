package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {

	public TalonFX FeederFalcon = new TalonFX(20, "canivore");
    public TalonFXConfiguration FeederFXConfig = new TalonFXConfiguration();
    private DigitalInput input = new DigitalInput(0);


	public Feeder() {
        /** Shooter Motor Configuration */
        /* Motor Inverts and Neutral Mode */
		FeederFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        FeederFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        /* Current Limiting */
        FeederFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        FeederFXConfig.CurrentLimits.SupplyCurrentLimit = 20;
        FeederFXConfig.CurrentLimits.SupplyCurrentThreshold = 40;
        FeederFXConfig.CurrentLimits.SupplyTimeThreshold = 0.1;

        /* PID Config */
        FeederFXConfig.Slot0.kP = 0.2;
        FeederFXConfig.Slot0.kI = 0;
        FeederFXConfig.Slot0.kD = 0;

        /* Open and Closed Loop Ramping */
        FeederFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.05;
        FeederFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.05;

        FeederFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.05;
        FeederFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.05;

        // Config Motor
        FeederFalcon.getConfigurator().apply(FeederFXConfig);
        FeederFalcon.getConfigurator().setPosition(0.0);
	}

    public boolean readInput() {
        return input.get();
    }

	public void setSpeed(double speed) {
        this.FeederFalcon.set(speed);
	}

	public double getCurrentDraw() {
		return this.FeederFalcon.getSupplyCurrent().getValueAsDouble();
	}

	public void resetShooterEncoder() {
        try {
			FeederFalcon.getConfigurator().setPosition(0.0);
        }
        catch (Exception e) {
            DriverStation.reportError("Shooter.resetShooterEncoders exception.  You're Screwed! : " + e.toString(), false);
        }
	}

	public void updateDashboard() {
		SmartDashboard.putNumber("Feeder Current", this.getCurrentDraw());
        SmartDashboard.putBoolean("IR Break Beam", this.readInput());
	}
}