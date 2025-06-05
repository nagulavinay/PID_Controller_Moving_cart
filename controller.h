class Controller {
public:
  /**
   * @brief Computes the control output based on the given error.
   *
   * This method computes the control output based on the error between
   * the desired reference value and the current system state.
   *
   * @param error The error (difference between reference value and current
   * state).
   * @return The control output computed by the controller.
   */
  virtual double output(double error) = 0;
  /**
   * @brief Updates the controller parameters dynamically.
   *
   * This method allows for dynamic updating of controller parameters,
   * such as proportional, integral, and derivative gains.
   *
   * @param kp The proportional gain.
   * @param kd The derivative gain.
   * @param ki The integral gain.
   */
  virtual void update_params(double kp, double kd, double ki) = 0;

  /**
   * @brief Sets the clamping limits for the control output.
   *
   * This method allows setting upper and lower limits for the control output
   * to prevent excessive control action.
   *
   * @param max The maximum allowed control signal.
   * @param min The minimum allowed control signal.
   */
  virtual void setClamp(double max, double min) = 0;
};

/**
 * @brief Implementation of a PID (Proportional-Integral-Derivative) controller.
 *
 * The PIDController class implements a PID controller for controlling the
 * inverted pendulum system. It computes control signals based on proportional,
 * integral, and derivative terms, and provides methods for updating controller
 * parameters and setting clamping limits for the control output.
 */
class PIDController : public Controller {
  //@todo Add private members for PIDController class
public:
  /**
   * @brief Default constructor.
   *
   * Initializes the PID controller with default proportional, derivative,
   * and integral gains.
   */
  double kp = 0.0;
  double kd = 0.0;
  double ki = 0.0;

  ///@todo Intialize paramaters for discrete PID controller
  double N = 1;
  double Ts = 0.0001;
  double a0, a1, a2, b0, b1, b2;
  double e_prev[2]={1000000,-1000000};
  double u_prev[2]={0.0,0.0};
  int step_count = 0;
  double u = 0.0;

  

  PIDController();

  /**
   * @brief Computes the control output based on the given error.
   *
   * This method implements the PID control algorithm to compute the control
   * output based on the error between the desired reference value and the
   * current system state.
   *
   * @param error The error (difference between reference value and current
   * state).
   * @return The control output computed by the PID controller.
   */
  double output(double error);
  /**
   * @brief Updates the controller parameters (gains).
   *
   * This method allows for dynamic updating of the PID controller's gains
   * (proportional, derivative, and integral gains).
   *
   * @param kp The new proportional gain.
   * @param kd The new derivative gain.
   * @param ki The new integral gain.
   */
  void update_params(double kp, double kd, double ki);
  /**
   * @brief Sets the clamping limits for the control output.
   *
   * This method allows setting upper and lower limits for the control output
   * to prevent excessive control action.
   *
   * @param max The maximum allowed control signal.
   * @param min The minimum allowed control signal.
   */
  void setClamp(double max, double min);
};
