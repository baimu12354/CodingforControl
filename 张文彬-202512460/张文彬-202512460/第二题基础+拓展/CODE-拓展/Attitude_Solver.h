#ifndef ATTITUDE_SOLVER_H
#define ATTITUDE_SOLVER_H

/* 原有陀螺校准、Mahony 四元数更新及欧拉角换算。 */
void Attitude_Solver_Calibrate(void);
void Attitude_Solver_Update(float dt);

#endif /* ATTITUDE_SOLVER_H */
