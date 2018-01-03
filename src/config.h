#ifndef CONFIG_HEAD
#define CONFIG_HEAD

// TODO: Set the timestep length and duration
extern size_t N;

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
extern size_t x_start;
extern size_t y_start;
extern size_t psi_start;
extern size_t v_start;
extern size_t cte_start;
extern size_t epsi_start;
extern size_t delta_start;
extern size_t a_start;

#endif //CONFIG_HEAD
