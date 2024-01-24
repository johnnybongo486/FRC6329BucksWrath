package frc.robot.commands.Drivetrain;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class VisionAlignIntake extends Command {    
    private Swerve s_Swerve;    
    private Boolean robotCentricSup;

    private double tx = 0;
    private double ta = 0;

    private final PIDController angleController = new PIDController(0.01, 0, 0);  // 0.012 0.2     0.0002
    private double targetAngle = 0;
    private final PIDController distanceController = new PIDController(0.05, 0, 0);
    private double targetArea = 6;  // what is the area when we pick up gp?

    public VisionAlignIntake(Swerve s_Swerve, Boolean robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        addRequirements(RobotContainer.rearLimelight);

        this.robotCentricSup = robotCentricSup;
    }

    public void initialize() {
        tx = RobotContainer.rearLimelight.getX();
        ta = RobotContainer.rearLimelight.getArea();

        angleController.setTolerance(0.05);  // needs to be checked
        distanceController.setTolerance(0.05);  // needs to be checked
    }
    
    @Override
    public void execute() {
        // Find target location
        tx = RobotContainer.rearLimelight.getX();
        ta = RobotContainer.rearLimelight.getArea();

        double rotationVal = angleController.calculate(tx,targetAngle);
        double strafeVal = -angleController.calculate(tx,targetAngle);
        double translationVal = -distanceController.calculate(ta,targetArea);

        /* Drive */
        if (ta < 4) {
            s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup, 
            true
            );  
        }
        
        else {

        }

    }

    public boolean isFinished() {
		return ta > 4;
	}

	// Called once after isFinished returns true
	protected void end() {
        
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
