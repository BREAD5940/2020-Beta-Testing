package frc.robot.subsystems.superstructure
//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
//yeet delete ^^^^^
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.subsystems.climb.ClimbSubsystem
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.inInches
import org.ghrobotics.lib.mathematics.units.inches

object Superstructure : FalconSubsystem() {

    class PresetCommand(vararg commands: FalconCommand) : SequentialCommandGroup() {
        //Used to tell if running
        init {
            addCommands(*commands)
        }
    }

    fun goToPreset(elevatorHeightPreset : SIUnit <Meter> , proximalAnglePreset : SIUnit <Radian> , wristAnglePreset : SIUnit <Radian>, habYeetPreset : SIUnit <Meter>) {

        var elevater = Elevator.master.encoder.position
        var proximal = Proximal.ProximalMaster.encoder.position
        var wrist = Wrist.wristMotor.encoder.position
        var habyeet = ClimbSubsystem.habClimberStilt.encoder.position


        if(elevater > 40.inches && elevatorHeightPreset > 40.inches){
            //if high and going higher
            //dose not matter no risk of hitting things
            PresetCommand(Proximal.ProximalPreset(proximalAnglePreset),
            ElevatorPresets.ElevatorGoToPreset(elevatorHeightPreset),
            Wrist.WristPreset(wristAnglePreset), ClimbSubsystem.StiltSend(habYeetPreset)).schedule()
        }
        // if high and going low
        else if(elevater > 40.inches){
            //proximal then wrist then elevator
            PresetCommand(Proximal.ProximalPreset(proximalAnglePreset), Wrist.WristPreset(wristAnglePreset), ElevatorPresets.ElevatorGoToPreset(elevatorHeightPreset)).schedule()
        }
        //low and going lower/higher heheh
        else {
            //elevator then proximal then wrist
            PresetCommand(ElevatorPresets.ElevatorGoToPreset(elevatorHeightPreset), Proximal.ProximalPreset(proximalAnglePreset), Wrist.WristPreset(wristAnglePreset)).schedule()
        }
    }
}