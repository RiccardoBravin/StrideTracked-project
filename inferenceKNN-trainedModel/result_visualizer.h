/*
  Contains the function that is called by the C++ code to visualize the result of inference using 
  the Arduino's built-in LED. 
  Also contains other helpful functions
*/

#ifndef RICCARDO_RESULT_VISUALIZER_H_
#define RICCARDO_RESULT_VISUALIZER_H_



// Called by the main loop to turn on a led color corresponding to the class number
void class_to_led(int class_label, bool log_silence = false);

#endif  // RICCARDO_RESULT_VISUALIZER_H_
