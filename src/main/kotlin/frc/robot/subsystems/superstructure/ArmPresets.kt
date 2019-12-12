package frc.robot.subsystems.superstructure
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.subsystems.climb.ClimbSubsystem
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.inches

object ArmPresets {
    //Elevator, proximal then wrist (order for commands
    //TODO Put in real presets!!!!!
    var hab3 = false
    var hab2 = false

    fun noHab(){
        var hab3 = false
        var hab2 = false
    }
                //35 inches is elevator true "0"

        fun cargoLow(){
               Superstructure.goToPreset(25.inches, 90.degrees, 10.degrees, 0.0.inches)
        }
        fun cargoMid(){
            Superstructure.goToPreset(50.inches, 90.degrees, 10.degrees, 0.0.inches)
        }
        fun cargoHigh(){
            Superstructure.goToPreset(75.inches, 90.degrees, 10.degrees, 0.0.inches)
        }
        fun hatchLow(){
            Superstructure.goToPreset(25.inches, 90.degrees, 0.degrees, 0.0.inches)
        }
        fun hatchMid(){
            Superstructure.goToPreset(50.inches, 90.degrees, 0.degrees, 0.0.inches)
        }
        fun hatchHigh(){
            Superstructure.goToPreset(75.inches, 90.degrees, 0.degrees, 0.0.inches)
        }
        fun stowed(){
            Superstructure.goToPreset(15.inches, 90.degrees, 0.degrees, 0.0.inches)
        }
        fun hab3Prep(){
            Superstructure.goToPreset(35.inches, (-5).degrees, 93.degrees, 0.0.inches) //right
            hab3 = true
            hab2 = false
        }
        fun hab2Prep(){
            Superstructure.goToPreset(15.inches, (-5).degrees, 93.degrees, 0.0.inches) //tune
            hab3 = false
            hab2 = true

        }
        fun habYEET(){
            if(hab3) {
                ClimbSubsystem.habClimberStilt.setPosition(20.inches) //change to actual height
                Superstructure.goToPreset(10.inches, (-5).degrees, 93.degrees, 0.0.inches) //good angle bad height
            }
            if(hab2) {
                ClimbSubsystem.habClimberStilt.setPosition(20.inches) //change to actual height
                Superstructure.goToPreset(8.inches, (-5).degrees, 93.degrees, 0.0.inches) // good angle bad height
            }
        }

    }
