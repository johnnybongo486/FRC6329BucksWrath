package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drivetrain.Drive2Feet;
import frc.robot.subsystems.Swerve;

public class VisionIntakeCommandGroup extends SequentialCommandGroup {

    private Swerve s_Swerve;    
    
    public VisionIntakeCommandGroup(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        addCommands(new VisionAlignIntake(s_Swerve, true).andThen(new Drive2Feet(s_Swerve, true).withTimeout(1)));
    }

}
