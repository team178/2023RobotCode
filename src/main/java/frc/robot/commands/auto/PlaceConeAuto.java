package frc.robot.commands.auto;

import frc.robot.commands.Autos;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

public class PlaceConeAuto extends AutoCommand {
    public PlaceConeAuto(Arm arm, Claw claw) {
        this.addCommands(Autos.placeHigh(arm, claw));
    }
}
