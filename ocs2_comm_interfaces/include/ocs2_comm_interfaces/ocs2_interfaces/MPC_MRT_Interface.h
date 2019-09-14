#pragma once

#include <ocs2_comm_interfaces/ocs2_interfaces/MRT_BASE.h>

#include <chrono>
#include <condition_variable>
#include <csignal>
#include <ctime>
#include <iostream>
#include <string>
#include <thread>

#include <ocs2_comm_interfaces/SystemObservation.h>
#include <ocs2_core/logic/machine/HybridLogicRulesMachine.h>
#include <ocs2_mpc/MPC_BASE.h>

// For RosWarnStream, can be removed if std::cout is used instead
#include <ros/ros.h>

namespace ocs2 {

/**
 * A lean ROS independent interface to OCS2. In incorporates the functionality of the MPC and the MRT (trajectory tracking) modules.
 * Please refer to ocs2_double_integrator_example for a minimal example and tests
 * @tparam STATE_DIM
 * @tparam INPUT_DIM
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class MPC_MRT_Interface final : public MRT_BASE<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Base = MRT_BASE<STATE_DIM, INPUT_DIM>;
  using policy_data_t = typename Base::policy_data_t;
  using command_data_t = typename Base::command_data_t;

  using Ptr = std::shared_ptr<MPC_MRT_Interface<STATE_DIM, INPUT_DIM>>;

  using mpc_t = MPC_BASE<STATE_DIM, INPUT_DIM>;

  using scalar_t = typename mpc_t::scalar_t;
  using scalar_array_t = typename mpc_t::scalar_array_t;
  using scalar_array2_t = typename mpc_t::scalar_array2_t;
  using size_array_t = typename mpc_t::size_array_t;
  using state_vector_t = typename mpc_t::state_vector_t;
  using state_vector_array_t = typename mpc_t::state_vector_array_t;
  using state_vector_array2_t = typename mpc_t::state_vector_array2_t;
  using input_vector_t = typename mpc_t::input_vector_t;
  using input_vector_array_t = typename mpc_t::input_vector_array_t;
  using input_vector_array2_t = typename mpc_t::input_vector_array2_t;
  using controller_t = typename mpc_t::controller_t;
  using controller_ptr_array_t = typename mpc_t::controller_ptr_array_t;
  using input_state_matrix_t = typename mpc_t::input_state_matrix_t;
  using input_state_matrix_array_t = typename mpc_t::input_state_matrix_array_t;
  using dynamic_vector_t = typename mpc_t::dynamic_vector_t;

  using cost_desired_trajectories_t = typename mpc_t::cost_desired_trajectories_t;
  using mode_sequence_template_t = typename mpc_t::mode_sequence_template_t;

  using system_observation_t = SystemObservation<STATE_DIM, INPUT_DIM>;

  using state_linear_interpolation_t = EigenLinearInterpolation<state_vector_t>;
  using input_linear_interpolation_t = EigenLinearInterpolation<input_vector_t>;

  /**
   * Constructor
   * @param [in] mpc the underlying MPC class to be used
   * @param [in] logicRules (optional)
   */
  explicit MPC_MRT_Interface(mpc_t& mpc, std::shared_ptr<HybridLogicRules> logicRules = nullptr);

  /**
   * Destructor.
   */
  virtual ~MPC_MRT_Interface() = default;

  void resetMpcNode(const cost_desired_trajectories_t& initCostDesiredTrajectories) override;

  void setCurrentObservation(const system_observation_t& currentObservation) override;

  /**
   * Set new target trajectories to be tracked.
   * It is safe to set a new value while the MPC optimization is running
   * @param targetTrajectories
   */
  void setTargetTrajectories(const cost_desired_trajectories_t& targetTrajectories);

  /**
   * Set a new mode sequence template
   * It is safe to set a new value while the MPC optimization is running
   * @param [in] modeSequenceTemplate
   */
  void setModeSequence(const mode_sequence_template_t& modeSequenceTemplate);

  /**
   * Advance the mpc module for one iteration.
   * The evaluation methods can be called while this method is running.
   * They will evaluate the control law that was up-to-date at the last updatePolicy() call
   */
  void advanceMpc();

  /**
   * @brief Access the solver's internal value function
   * @param time query time
   * @param state query state
   * @return value of the given state at the given time
   */
  scalar_t getValueFunction(scalar_t time, const state_vector_t& state);

  /**
   * @brief Calculates the state derivative of the value function
   * @param [in] time the query time
   * @param [out] Vx partial derivative of the value function at requested time at nominal state
   */
  void getValueFunctionStateDerivative(scalar_t time, const state_vector_t& state, state_vector_t& Vx);

  /**
   * @brief getLinearFeedbackGain retrieves K matrix from solver
   * @param [in] time
   * @param [out] K
   */
  void getLinearFeedbackGain(scalar_t time, input_state_matrix_t& K);

  /**
   * @brief Computes the Lagrange multiplier related to the state-input constraints
   * @param [in] time: query time
   * @param [in] state: query state
   * @param [out] nu: the Lagrange multiplier
   */
  void getStateInputConstraintLagrangian(scalar_t time, const state_vector_t& state, dynamic_vector_t& nu) const;

 protected:
  /**
   * @brief fillMpcOutputBuffers updates the *Buffer variables from the MPC object.
   * This method is automatically called by advanceMpc()
   * @param [in] mpcInitObservation: The observation used to run the MPC.
   * @param [in] mpc: A reference to the MPC instance.
   * @param [out] policyDataPtr: The policy data of the MPC.
   * @param [out] commandDataPtr: The command data of the MPC.
   */
  void fillMpcOutputBuffers(system_observation_t mpcInitObservation, const mpc_t& mpc, policy_data_t* policyDataPtr,
                            command_data_t* commandDataPtr);

 protected:
  mpc_t& mpc_;

  size_t numMpcIterations_;
  scalar_t maxDelay_ = 0;
  scalar_t meanDelay_ = 0;

  // MPC inputs
  system_observation_t currentObservation_;
  std::mutex observationMutex_;
};

}  // namespace ocs2

#include "implementation/MPC_MRT_Interface.h"