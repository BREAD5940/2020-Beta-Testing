package frc.robot.subsystems.superstructure
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.inches
class ElevatorPresets : FalconCommand(Elevator) {
    //TODO Change to real presets
class Hatch {
        public fun low() {
            Elevator.master.setPosition(10.inches)
        }
        fun mid() {
            Elevator.master.setPosition(20.inches)
        }
        public fun high() {
            Elevator.master.setPosition(30.inches)
        }
}
 class Cargo {

     public fun low() {
         Elevator.master.setPosition(11.inches)
     }
     fun mid() {
         Elevator.master.setPosition(22.inches)
     }
     public fun high() {
         Elevator.master.setPosition(33.inches)
     }
 }


}