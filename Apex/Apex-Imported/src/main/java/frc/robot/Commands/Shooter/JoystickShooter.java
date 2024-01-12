package frc.robot.Commands.Shooter;

import frc.robot.*;

import edu.wpi.first.wpilibj.command.Command;

public class JoystickShooter extends Command {

	private int positionIncrement = 100;

	public JoystickShooter() {
		requires(Robot.Shooter);

	}

	// Called just before this Command runs the first time
	protected void initialize() {

	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {

		// joystick control
		// double signal = -Robot.oi.getOperatorRightStick();
		double signal = 0;
        
		Robot.Shooter.incrementTargetVelocity((int) (signal * positionIncrement));

		Robot.Shooter.velocityControl();
		
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}