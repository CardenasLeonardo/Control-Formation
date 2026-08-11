string robot_id

float64[] a
float64[] b
float64[] c

float64 v_goal
float64 w_goal

float64 v_star
float64 w_star

bool is_deadlock

# Estrategia de bordeo (solo válidos si is_deadlock=true y has_edge=true)
int32 search_dir

bool    has_edge
float64 edge_v_sel
float64 edge_w_sel
float64 edge_v_other
float64 edge_w_other

float64 v_z
float64 v_z_previous
