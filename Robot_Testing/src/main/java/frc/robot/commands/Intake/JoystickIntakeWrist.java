package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class JoystickIntakeWrist extends Command {

	private int positionIncrement = 10;
    
    public JoystickIntakeWrist() {
        addRequirements(RobotContainer.intakeWrist);
    }
	// Called just before this Command runs the first time
	public void initialize() {

	}

	// Called repeatedly when this Command is scheduled to run
	public void execute() {

		// joystick control
        double signal = RobotContainer.intakeWrist.JoyStickIntakeWrist();

        RobotContainer.intakeWrist.incrementTargetPosition((int) (signal * positionIncrement));

		RobotContainer.intakeWrist.motionMagicControl();

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

