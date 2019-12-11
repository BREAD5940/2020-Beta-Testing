package frc.robot.subsystems.superstructure
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.inches

object ArmPresets {
    //Elevator, proximal then wrist (order for commands
    //TODO Put in real presets!!!!!
        fun cargoLow(){
               Superstructure.goToPreset(25.inches, 90.degrees, 10.degrees)
        }
        fun cargoMid(){
            Superstructure.goToPreset(50.inches, 90.degrees, 10.degrees)
        }
        fun cargoHigh(){
            Superstructure.goToPreset(75.inches, 90.degrees, 10.degrees)
        }
        fun hatchLow(){
            Superstructure.goToPreset(25.inches, 90.degrees, 0.degrees)
        }
        fun hatchMid(){
            Superstructure.goToPreset(50.inches, 90.degrees, 0.degrees)
        }
        fun hatchHigh(){
            Superstructure.goToPreset(75.inches, 90.degrees, 0.degrees)
        }
        fun stowed(){
            Superstructure.goToPreset(15.inches, 90.degrees, 1.degrees)
        }
    }
