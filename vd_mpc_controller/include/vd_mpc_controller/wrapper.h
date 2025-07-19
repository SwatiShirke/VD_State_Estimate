#include <Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include "acados/utils/math.h"
#include "ackerman_model_model.h"
#include "acados_solver_ackerman_model.h"

#define NX              ACKERMAN_MODEL_NX
#define NZ              ACKERMAN_MODEL_NZ
#define NU              ACKERMAN_MODEL_NU
#define NP              ACKERMAN_MODEL_NP
#define NBX             ACKERMAN_MODEL_NBX
#define NBX0            ACKERMAN_MODEL_NBX0
#define NBU             ACKERMAN_MODEL_NBU
#define NSBX            ACKERMAN_MODEL_NSBX
#define NSBU            ACKERMAN_MODEL_NSBU
#define NSH             ACKERMAN_MODEL_NSH
#define NSH0            ACKERMAN_MODEL_NSH0
#define NSG             ACKERMAN_MODEL_NSG
#define NSPHI           ACKERMAN_MODEL_NSPHI
#define NSHN            ACKERMAN_MODEL_NSHN
#define NSGN            ACKERMAN_MODEL_NSGN
#define NSPHIN          ACKERMAN_MODEL_NSPHIN
#define NSPHI0          ACKERMAN_MODEL_NSPHI0
#define NSBXN           ACKERMAN_MODEL_NSBXN
#define NS              ACKERMAN_MODEL_NS
#define NS0             ACKERMAN_MODEL_NS0
#define NSN             ACKERMAN_MODEL_NSN
#define NG              ACKERMAN_MODEL_NG
#define NBXN            ACKERMAN_MODEL_NBXN
#define NGN             ACKERMAN_MODEL_NGN
#define NY0             ACKERMAN_MODEL_NY0
#define NY              ACKERMAN_MODEL_NY
#define NYN             ACKERMAN_MODEL_NYN
//#define N               ACKERMAN_MODEL_N
#define NH              ACKERMAN_MODEL_NH
#define NHN             ACKERMAN_MODEL_NHN
#define NH0             ACKERMAN_MODEL_NH0
#define NPHI0           ACKERMAN_MODEL_NPHI0
#define NPHI            ACKERMAN_MODEL_NPHI
#define NPHIN           ACKERMAN_MODEL_NPHIN
#define NR              ACKERMAN_MODEL_NR


const int N = ACKERMAN_MODEL_N;
namespace nmpc_control_nodelet
{
    static constexpr int kStateSize = ACKERMAN_MODEL_NX;
    static constexpr int kInputSize = ACKERMAN_MODEL_NU;
    static constexpr int kSamples = N;
    static constexpr int yRefSize = ACKERMAN_MODEL_NX + ACKERMAN_MODEL_NU;

    struct solver_output
    {
        // The Eigen Maps initialized in the class can directly change these values below
        // without worrying about transforming between matrices and arrays
        // the relevant sections of the arrays can then be passed to the solver
        double status, KKT_res, cpu_time;
        double u0[NU];
        double u1[NU];
        double x1[NX];
        double x2[NX];
        double x4[NX];
        double xi[NU];
        double ui[NU];
        double u_out[NU * (N)];
        double x_out[NX * (N)];


    };
    struct solver_input
    {
      double x0[NX];
      double x[NX * (N)];
      double u[NU * N];
      double yref[(NX + NU) * N];
      double yref_e[(NX + NU)];
      #if (NY > 0)
      
        double W[NY * NY];
      #endif 
      
      double WN[NX * NX];
    };

    extern solver_input acados_in;
    extern solver_output acados_out;

    class NMPCWrapper
    {
    public:
    NMPCWrapper();
    NMPCWrapper(const Eigen::VectorXd Q_, const Eigen::VectorXd R_,
              const Eigen::VectorXd lbu_, const Eigen::VectorXd ubu_);

    void setTrajectory(const Eigen::Ref<const Eigen::Matrix<double, kStateSize, kSamples>> states,
                       const Eigen::Ref<const Eigen::Matrix<double, kInputSize, kSamples>> inputs);
    
    bool prepare(const Eigen::Ref<const Eigen::Matrix<double, kStateSize, 1>> state);
    bool update(const Eigen::Ref<const Eigen::Matrix<double, kStateSize, 1>> state);
    void getStates(Eigen::Matrix<double, kStateSize, kSamples> &return_state);
    void getInputs(Eigen::Matrix<double, kInputSize, kSamples> &return_input);

    void setMass(double mass);
    void setGravity(double gravity);
    void initStates();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
    ackerman_model_solver_capsule *acados_ocp_capsule;
    ocp_nlp_in *nlp_in;
    ocp_nlp_out *nlp_out;
    //ocp_nlp_out *sens_out;
    ocp_nlp_solver *nlp_solver;
    void *nlp_opts;
    ocp_nlp_plan_t *nlp_solver_plan;
    ocp_nlp_config *nlp_config;
    ocp_nlp_dims *nlp_dims;
    double *new_time_steps;
    int status;
    double mass_;
    double gravity_;
    bool acados_is_prepared_{false};
    int acados_status;
    double *initial_state;

    Eigen::Map<Eigen::Matrix<double, yRefSize, kSamples, Eigen::ColMajor>> acados_reference_states_{acados_in.yref};
    Eigen::Map<Eigen::Matrix<double, kStateSize, 1, Eigen::ColMajor>> acados_initial_state_{acados_in.x0};
    Eigen::Map<Eigen::Matrix<double, yRefSize, 1, Eigen::ColMajor>> acados_reference_end_state_{acados_in.yref_e};
    Eigen::Map<Eigen::Matrix<double, kStateSize, kSamples, Eigen::ColMajor>> acados_states_in_{acados_in.x};
    Eigen::Map<Eigen::Matrix<double, kInputSize, kSamples, Eigen::ColMajor>> acados_inputs_in_{acados_in.u};
    Eigen::Map<Eigen::Matrix<double, kStateSize, kSamples, Eigen::ColMajor>> acados_states_{acados_out.x_out};
    Eigen::Map<Eigen::Matrix<double, kInputSize, kSamples, Eigen::ColMajor>> acados_inputs_{acados_out.u_out};
    Eigen::Matrix<real_t, kInputSize, 1> kVDInput_ = (Eigen::Matrix<real_t, kInputSize, 1>() << 0,0,0).finished();
    };

}// namespace control nodelet ends here