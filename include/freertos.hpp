#ifndef FREERTOS_HPP
#define FREERTOS_HPP

namespace freertos {

void createMicroRosTask();
void createGripperMotorTasks();
void createStepperMotorTasks();
void createMsgQueues();

} // namespace freertos
#endif // FREERTOS_HPP