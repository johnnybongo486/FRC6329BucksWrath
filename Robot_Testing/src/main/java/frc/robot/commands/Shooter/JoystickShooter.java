package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class JoystickShooter extends Command {

	private int positionIncrement = 10;
    
    public JoystickShooter() {
        addRequirements(RobotContainer.leftShooter);
        addRequirements(RobotContainer.rightShooter);
    }
	// Called just before this Command runs the first time
	public void initialize() {

	}

	// Called repeatedly when this Command is scheduled to run
	public void execute() {

		// joystick control
        double leftSignal = RobotContainer.leftShooter.joystickShooter();
        double rightSignal = RobotContainer.rightShooter.joystickShooter();

        RobotContainer.leftShooter.incrementTargetVelocity((int) (leftSignal * positionIncrement));
        RobotContainer.rightShooter.incrementTargetVelocity((int) (rightSignal * positionIncrement));

		RobotContainer.leftShooter.velocityControl();
        RobotContainer.rightShooter.velocityControl();

	}

	// Make this return true when this Command no longer needs to run execute()
	public boolean isFinished() {
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
