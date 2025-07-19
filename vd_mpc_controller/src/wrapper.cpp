#include "vd_mpc_controller/wrapper.h"

//common namespace nmpc_control_nodelet
namespace nmpc_control_nodelet
{
solver_input acados_in;
solver_output acados_out;

NMPCWrapper::NMPCWrapper()
{
// create solver capsule 
acados_ocp_capsule = ackerman_model_acados_create_capsule();
new_time_steps = NULL;
status = ackerman_model_acados_create_with_discretization(acados_ocp_capsule, N, new_time_steps); //capsule, N steps, new time step

// check staus 
if (status) {
    printf("ackerman_model_acados_create_with_discretization() returned status %d. Exiting.\n", status);
    exit(1);
  }

//set configs
nlp_config = ackerman_model_acados_get_nlp_config(acados_ocp_capsule);
nlp_dims = ackerman_model_acados_get_nlp_dims(acados_ocp_capsule);
nlp_in = ackerman_model_acados_get_nlp_in(acados_ocp_capsule);
nlp_out = ackerman_model_acados_get_nlp_out(acados_ocp_capsule);
nlp_solver = ackerman_model_acados_get_nlp_solver(acados_ocp_capsule);
nlp_opts = ackerman_model_acados_get_nlp_opts(acados_ocp_capsule);

//not sure if needed 
Eigen::Matrix<double, kStateSize, 1> VD_state(Eigen::Matrix<double, kStateSize, 1>::Zero());
// hover_state(6) = 1.0;

//Q: kHover not defined before, defined in init; but constructor would be called first
//Q: ocp_nlp_constraints_model_set - did not find the definition
//initialize states x and xN and input u
acados_initial_state_ = VD_state.template cast<double>();
acados_states_ = VD_state.replicate(1, kSamples).template cast<double>();
acados_inputs_ = kVDInput_.replicate(1, kSamples).template cast<double>();
ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", acados_in.x0);
ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", acados_in.x0);

std::cout << "I am at bp 1"<< '\n';

//initialize references y and yN.
acados_reference_states_.block(0, 0, kStateSize, kSamples) = VD_state.replicate(1, kSamples).template cast<double>();
acados_reference_states_.block(kStateSize, 0, kInputSize, kSamples) = kVDInput_.replicate(1, kSamples);
// std::cout << acados_reference_states_ << '\n';
// std::cout << "" << '\n';
// std::cout << "" << '\n';
// std::cout << acados_reference_end_state_ << '\n';
acados_reference_end_state_.segment(0, kStateSize) = VD_state.template cast<double>();
}


void NMPCWrapper::initStates()
{
Eigen::Matrix<double, kStateSize, 1> VD_state(Eigen::Matrix<double, kStateSize, 1>::Zero());
  //hover_state(6) = 1.0;
  kVDInput_ = (Eigen::Matrix<real_t, kInputSize, 1>() << 0.0, 0.0, 0.0).finished();

  // initialize states x and xN and input u.
  acados_initial_state_ = VD_state.template cast<double>();
  acados_states_ = VD_state.replicate(1, kSamples).template cast<double>();
  acados_inputs_ = kVDInput_.replicate(1, kSamples).template cast<double>();

  ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", acados_in.x0);
  ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", acados_in.x0);

  // initialize references y and yN.
  std::cout << "I am at bp 2"<< '\n';
  acados_reference_states_.block(0, 0, kStateSize, kSamples) = VD_state.replicate(1, kSamples).template cast<double>();
  acados_reference_states_.block(kStateSize, 0, kInputSize, kSamples) = kVDInput_.replicate(1, kSamples);
  acados_reference_end_state_.segment(0, kStateSize) = VD_state.template cast<double>();

}

void NMPCWrapper::setTrajectory(const Eigen::Ref<const Eigen::Matrix<double, kStateSize, kSamples>> states,
                                const Eigen::Ref<const Eigen::Matrix<double, kInputSize, kSamples>> inputs)
{
  std::cout << "I am at bp 3"<< '\n';
  acados_reference_states_.block(0, 0, kStateSize, kSamples) = states.block(0, 0, kStateSize, kSamples).template cast<double>();
  acados_reference_states_.block(kStateSize, 0, kInputSize, kSamples) = inputs.block(0, 0, kInputSize, kSamples).template cast<double>();
  acados_reference_end_state_.segment(0, kStateSize) = states.col(kSamples - 1).template cast<double>();
}

bool NMPCWrapper::prepare(const Eigen::Ref<const Eigen::Matrix<double, kStateSize, 1>> state)
{
  acados_states_ = state.replicate(1, kSamples).template cast<double>();
  acados_inputs_ = kVDInput_.replicate(1, kSamples).template cast<double>();
  for (int i = 0; i <= N; i++) {
      ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", acados_out.x_out + i * NX);
  }
  for (int i = 0; i < N; i++) {
      ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", acados_out.u_out + i * NU);
  }
  return true;
}


bool NMPCWrapper::update(const Eigen::Ref<const Eigen::Matrix<double, kStateSize, 1>> state)
{
  // This function provides as argument a NX by 1 vector which is the state
  // doing preparation step, this sets the solver to feedback and prepare phase
  int rti_phase = 0;  // zero sets rti_pahse to both prepare and feedback
  ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "rti_phase", &rti_phase);
  acados_is_prepared_ = true;

  // Setting initial state
  acados_initial_state_ = state.template cast<double>();

  ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, 0, "x", acados_in.x0);
  ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", acados_in.x0);
  ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", acados_in.x0);

  // for loop to set yref
  // loop over horizon and assign to each shooting node a segment of the acados_in.yref
  for (int i = 0; i < N; i++) {
      // sends pointer to element yref[i*NY]
      ackerman_model_acados_update_params(acados_ocp_capsule, i, acados_in.yref + i * yRefSize, NP);
       }
    ackerman_model_acados_update_params(acados_ocp_capsule, N, acados_in.yref_e, yRefSize);

  acados_status = ackerman_model_acados_solve(acados_ocp_capsule);

  // getting solved states from acados
  for (int ii = 0; ii <= nlp_dims->N; ii++)
      ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "x", &acados_out.x_out[ii * NX]);
  for (int ii = 0; ii < nlp_dims->N; ii++)
      ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "u", &acados_out.u_out[ii * NU]);

  return true;
}


void NMPCWrapper::getStates(Eigen::Matrix<double, kStateSize, kSamples> &return_states)
{
  return_states = acados_states_.cast<double>();
}

void NMPCWrapper::getInputs(Eigen::Matrix<double, kInputSize, kSamples> &return_input)
{
  return_input = acados_inputs_.cast<double>();
}

void NMPCWrapper::setMass(double mass)
{
  mass_ = mass;
}
void NMPCWrapper::setGravity(double gravity)
{
  gravity_ = gravity;
}

}