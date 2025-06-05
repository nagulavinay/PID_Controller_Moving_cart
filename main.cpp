#include "controller.h"
#include "server.h"
#include "simulator.h"
#include <thread>

/**
 * @brief Main function to start the simulation and communication server.
 *
 * This function initializes the simulator object and
 * communication server. It then starts the simulation and communication
 * server threads, waits for them to finish.
 *
 * @return 0 on successful completion.
 */

int main() {

  Simulator sim;        ///< Simulator object
  CommServer comm(sim); ///< Communication server object with simulator object

  std::jthread sim_thread(&Simulator::run_simulator,
                          std::ref(sim)); ///< Start the simulation thread

  std::jthread comm_thread(
      &CommServer::start_server,
      std::ref(comm)); ///< Start the communication server thread

  sim_thread.join();
  comm_thread.join(); ///< Wait for the threads to finish

  return 0;
}
