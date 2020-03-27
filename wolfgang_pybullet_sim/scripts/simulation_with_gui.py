from src.wolfgang_pybullet_sim.ros_interface import ROSInterface
from src.wolfgang_pybullet_sim.simulation import Simulation

if __name__ == "__main__":
    simulation = Simulation(True)
    ros_interace = ROSInterface(simulation)
    ros_interace.run_simulation()