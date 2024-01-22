package frc.robot.commands.Drivetrain;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;


public class VisionAlignIntake extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private Boolean robotCentricSup;
    private double slowSpeed = 0.2;
    private double midSpeed = 0.5;
    private double elevatorHeight = 0;

    private double tx = 0;
    private double ty = 0;
    private double ta = 0;// needs to be checked

    private final PIDController angleController = new PIDController(0.012, 0.2, 0.0002);
    private double targetAngle = 0;
    private final PIDController distanceController = new PIDController(0.012, 0.2, 0.0002);
    private double targetArea = 0;  // what is the area when we pick up gp?

    public VisionAlignIntake(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, Boolean robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        addRequirements(RobotContainer.rearLimelight);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
    }

    public void initialize() {
        tx = RobotContainer.rearLimelight.getX();
        ty = RobotContainer.rearLimelight.getY();
        ta = RobotContainer.rearLimelight.getArea();

        angleController.setTolerance(5);  // needs to be checked
        distanceController.setTolerance(5);  // needs to be checked
    }
    
    @Override
    public void execute() {
        //TODO: I'm not sure if I disabled the elevator consideration here correctly.
        //elevatorHeight = RobotContainer.elevator.getCurrentPosition();

        // find target location
        tx = RobotContainer.rearLimelight.getX();
        ty = RobotContainer.rearLimelight.getY();
        ta = RobotContainer.rearLimelight.getArea();

        double rotationVal = angleController.calculate(tx,targetAngle);
        double strafeVal = angleController.calculate(tx,targetAngle);
        double translationVal = distanceController.calculate(ta,targetArea);

        /*if (elevatorHeight >= 30000) {
            translationVal = translationVal * slowSpeed;
            strafeVal = strafeVal * slowSpeed;
            rotationVal = rotationVal * slowSpeed;
        }

        else if (elevatorHeight > 5000 && elevatorHeight < 29999) {
            translationVal = translationVal * midSpeed;
            strafeVal = strafeVal * midSpeed;
            rotationVal = rotationVal * midSpeed;
        }

        else {}*/

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            robotCentricSup, 
            true
        );

    }
}
