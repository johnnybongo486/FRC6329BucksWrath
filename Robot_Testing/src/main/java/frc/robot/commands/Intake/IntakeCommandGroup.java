package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Shooter.IntakeRunFeeder;

public class IntakeCommandGroup extends SequentialCommandGroup {
    
    public IntakeCommandGroup() {
        addCommands(new SetIntakePosition(38).andThen(new RunIntake().alongWith(new IntakeRunFeeder())));
    }

}
