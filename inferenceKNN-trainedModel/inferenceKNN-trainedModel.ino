/* Copyright 2023 The TensorFlow Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

#include <TensorFlowLite.h>
#include <Arduino_LSM9DS1.h> //for IMU
#include <arduinoFFT.h>
#include <Arduino_KNN.h>

//#include <pt.h> //for multithreading


#include "constants.h"
#include "main_functions.h"
#include "model.h"  //contains model data
#include "result_visualizer.h"
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_log.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"


#define MAX_SAMPLES 512
#define SAMPLING_FREQUENCY 238

#define OUTPUT_CLASSES 128

#define BUTTON_PIN 13   //the number of the pushbutton pin
#define LED_PIN 13



namespace {
const tflite::Model* model = nullptr;
tflite::MicroInterpreter* interpreter = nullptr;
TfLiteTensor* input = nullptr;
TfLiteTensor* output = nullptr;

constexpr int kTensorArenaSize = 82*1024;  //con quessto funziona:  135*1024.    65:non funziona  |  90:funziona  | 82 funziona->76520bytes liberi->120examples per la Knn | 
// Keep aligned to 16 bytes for CMSIS
alignas(16) uint8_t tensor_arena[kTensorArenaSize];

}  // namespace


double data[6][MAX_SAMPLES];
double imagAux[6][MAX_SAMPLES];

//variabili per gestire training mode
bool trainMode = false;
int exampleToAdd = 5; //numero sample da aggiungere alla knn
int counter; //per fare il ciclo della train mode
int newLabel; //inserita da tastiera

arduinoFFT FFTs[6]; //array di oggetti FFTs
  
//---------------------------------------------------------------- KNN setup  ----------------------------------------------------------------//
// create KNN classifier, input will be array of 128 floats
KNNClassifier myKNN(128);


void setup() {

  //---------------------------------------------------------------- IMU setup  ----------------------------------------------------------------//
  if (!IMU.begin()) {
    MicroPrintf("Failed to initialize IMU!");
    while (1);
  }

  /******  FS  Full Scale         range 0:±2g | 1:±24g | 2: ±4g | 3: ±8g  (default=2)                                           ******
  ******                          range 0: 245 dps | 1: 500 dps | 2: 1000  dps | 3: 2000 dps                                    ******
  *******  ODR Output Data Rate   range 0:off | 1:10Hz | 2:50Hz | 3:119Hz | 4:238Hz | 5:476Hz, (default=3)(not working 6:952Hz) ******/
  IMU.setAccelFS(0);   //best accuracy at 0
  IMU.setAccelODR(4);  //best refreshrate at 6 (if Gyro OFF?)
  IMU.setGyroFS(0);
  IMU.setGyroODR(4);

  /*******************    For an improved accuracy run the DIY_Calibration_Accelerometer sketch first.     ****************
  ********************         Copy/Replace the lines below by the code output of the program              ****************/
  IMU.setAccelOffset(0, 0, 0);  //   uncalibrated
  IMU.setAccelSlope(1, 1, 1);   //   uncalibrated
  IMU.setGyroOffset(0, 0, 0);   // = uncalibrated
  IMU.setGyroSlope(1, 1, 1);    // = uncalibrated

  IMU.accelUnit = GRAVITY;         // GRAVITY=1 or  METERPERSECOND2=9.81
  IMU.gyroUnit = DEGREEPERSECOND;  //   DEGREEPERSECOND  RADIANSPERSECOND  REVSPERMINUTE  REVSPERSECOND

  

  //---------------------------------------------------------------- CNN setup  ----------------------------------------------------------------//
  tflite::InitializeTarget();

  // Map the model into a usable data structure. This doesn't involve any
  // copying or parsing, it's a very lightweight operation.
  model = tflite::GetModel(g_model);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    MicroPrintf("Model provided is schema version %d not equal to supported version %d.", model->version(), TFLITE_SCHEMA_VERSION);
    return;
  }

  // This pulls in all the operation implementations we need.
  // NOLINTNEXTLINE(runtime-global-variables)
  static tflite::AllOpsResolver resolver;

  // Build an interpreter to run the model with.
  static tflite::MicroInterpreter static_interpreter(model, resolver, tensor_arena, kTensorArenaSize);
  interpreter = &static_interpreter;

  // Allocate memory from the tensor_arena for the model's tensors.
  TfLiteStatus allocate_status = interpreter->AllocateTensors();
  if (allocate_status != kTfLiteOk) {
    MicroPrintf("AllocateTensors() failed");
    return;
  }

  // Obtain pointers to the model's input and output tensors.
  input = interpreter->input(0);
  output = interpreter->output(0);

  Serial.begin(9600); //inserito per prendere dati da input
  while (!Serial);

  //---------------------------------------------------------------- FFT setup  ----------------------------------------------------------------//

  //inizializzo FFTs objects by passing all reference 
  for(int i = 0; i < 6; i++){
    FFTs[i] = arduinoFFT(data[i], imagAux[i], MAX_SAMPLES, SAMPLING_FREQUENCY);
  }
  
  //---------------------------------------------------------------- Misc setup  ----------------------------------------------------------------//

  // initialize the LED as an output:
  pinMode(LED_PIN, OUTPUT);

}


void loop() {

  //define variables for IMU reads
  int16_t acc[3], gyro[3];

  MicroPrintf("Reading world data...\n");
  // Read samples until it gets to the Nsamples value
  int sample_count = 0;
  while (sample_count < MAX_SAMPLES) {

    if (IMU.accelAvailable() && IMU.gyroAvailable()) {
      //read both accelerometer and gyro data
      IMU.readRawAccelArr(acc);
      IMU.readRawGyroArr(gyro);

      //Converti i valori letti nell'intervallo desiderato per il modello
      data[0][sample_count] = (double)(acc[0]) / 16384; //accx
      data[1][sample_count] = (double)(acc[1]) / 16384; //accy
      data[2][sample_count] = (double)(acc[2]) / 16384; //accz
      data[3][sample_count] = (double)(gyro[0]) / 16384; //gyrox
      data[4][sample_count] = (double)(gyro[1]) / 16384; //gyroy
      data[5][sample_count] = (double)(gyro[2]) / 16384; //gyroz

      imagAux[0][sample_count] = 0;
      imagAux[1][sample_count] = 0;
      imagAux[2][sample_count] = 0;
      imagAux[3][sample_count] = 0;
      imagAux[4][sample_count] = 0;
      imagAux[5][sample_count] = 0;
      
      sample_count++;

      //DEBUG
      //Serial.print((double)(acc[2]) / 16384);
      //Serial.print(", ");
    }
  }

  //DEBUG
  /*Serial.println("");

  for (int j = 0; j < (MAX_SAMPLES >> 1); j++) {
      Serial.print(data[2][j]);
      Serial.print(", ");
  }

  Serial.println("");*/

  MicroPrintf("Performing FFT...\n");
  unsigned long startTimeFFT = millis();

  
  //Intrinsically the FFT object performs the transform on the data matrix
  for(int i = 0; i < 6; i++){
    FFTs[i].Compute(FFT_FORWARD);
    FFTs[i].ComplexToMagnitude();
  }

  //DEBUG
  /*for (int j = 0; j < (MAX_SAMPLES >> 1); j++) {
      Serial.print(static_cast<float>(data[0][j]));
      Serial.print(", ");
  }

  Serial.println("");*/
  
  MicroPrintf("Time for FFT %u", millis()-startTimeFFT);


  MicroPrintf("Starting inference of NN...");
  unsigned long startTimeInference = millis();

  int input_index = 0;
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < (MAX_SAMPLES >> 1); j++) {
      input->data.f[input_index] = static_cast<float>(data[i][j]);
      input_index++;
    }
  }


  
  // Run inference, and report any error
  TfLiteStatus invoke_status = interpreter->Invoke();
  if (invoke_status != kTfLiteOk) {
    MicroPrintf("Invoke failed on data array\n");
    return;
  }

  MicroPrintf("Time for inference: %u\n", millis()-startTimeInference);


  // Obtain the output from model's output tensor
  float *y = output->data.f;

  //DEBUG
  /*for(int i = 0; i < 128; i++){
    Serial.print(y[i]);
    Serial.print(", ");
  }*/

  Serial.println("");

  char inputStr[100] = ""; 
  Serial.readBytesUntil('\n', inputStr, 99);

  //if button is pressed, enter train mode
  if(inputStr[0] != 0){
    trainMode = true; //set train mode flag
    newLabel = inputStr[0] - '1' + 1;
    digitalWrite(LED_PIN, HIGH);

    counter = exampleToAdd;     //reset counter

    MicroPrintf("Welcome in the TRAIN MODE: Chosen label is: %d", newLabel);
    //delay(3000);
  }
  

  if(trainMode){ //TRAIN MODE
    myKNN.addExample(y, newLabel); //add example to KNN classifier

    MicroPrintf("Remaining examples to be added...: %d",counter);
    counter--;
    
    if(counter==0){ //if all examples have been added, exit train mode
      trainMode = false;
      //autoLabel++; //increment auto label for next training session
      MicroPrintf("Returning to INFERENCE MODE");
      //delay(3000);
    }
    

  }else{ //CLASSIFY MODE
    int personLabel = myKNN.classify(y, 10); //k=5. provo con k=sqrt(120)

    if(personLabel > 0){ //nota: registrare label>0
      MicroPrintf("Predicted class: %d\nConfidence: %f", personLabel, myKNN.confidence());
      class_to_led(personLabel, true);
    }
    else{
      MicroPrintf("Unknown person, did you train me?\n");
    }
    
    //OLD
    /*if(Serial.available()>0){
      //newLabel = readNumber(); //ora non serve piu, uso label che si incrementa
      trainMode=true;
      counter = exampleToAdd;
      MicroPrintf("Welcome in the TRAIN MODE: Auto label is: %d",autoLabel);
      delay(3000);
    }*/
  }
  
  //OLD
  //just checking the shape of y
  // MicroPrintf("Output probabilities:");
  // for(int i = 0; i < OUTPUT_CLASSES; i++){
  //   MicroPrintf("%d", i);
  //   MicroPrintf("%f", y[i]);
  // }

  //int class_index = 0;
  // for(int i = 1; i < OUTPUT_CLASSES; i++){
  //   if(y[i] > y[class_index]){
  //     class_index = i;
  //   }
  // }

  //MicroPrintf("La classe predetta è %d con probabilità %f", class_index, y[class_index]);

}

/*
// reads a number from the Serial Monitor
// expects new line
int readNumber() {
  String line;

  while (1) {
    if (Serial.available()) {
      char c = Serial.read();

      if (c == '\r') {
        // ignore
        continue;
      } else if (c == '\n') {
        break;
      }

      line += c;
    }
  }
}
*/
