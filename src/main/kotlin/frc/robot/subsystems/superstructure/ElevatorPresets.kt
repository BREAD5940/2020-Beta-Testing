package frc.robot.subsystems.superstructure
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.inches
class ElevatorPresets : FalconCommand(Elevator) {
        //TODO Change to real presets
class Hatch {
        fun low() {
            Elevator.master.setPosition(10.inches)
        }
        fun mid() {
            Elevator.master.setPosition(20.inches)
        }
        fun high() {
            Elevator.master.setPosition(30.inches)
        }
}
 class Cargo {

     fun low() {
         Elevator.master.setPosition(11.inches)
     }
     fun mid() {
         Elevator.master.setPosition(22.inches)
     }
     fun high() {
         Elevator.master.setPosition(33.inches)
     }
     fun zero() {
         Elevator.master.setPosition(0.inches)
     }
 }



}