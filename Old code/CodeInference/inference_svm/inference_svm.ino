#include <Arduino_LSM9DS1.h>

#include <TensorFlowLite.h>

#include "svm_q_model.hpp"
#include "classifier_SVM.h"

#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
//#include "tensorflow/lite/micro/micro_log.h" //in quella di Harvard copiare micro_log.h e .cpp (si trova in TFLite libreria)
#include "tensorflow/lite/micro/micro_error_reporter.h"
//#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"

//https://stackoverflow.com/questions/66664484/tensorflow-lite-on-arduino-nano-33-ble-didnt-find-op-for-builtin-opcode-expa

// Globals, used for compatibility with Arduino-style sketches.
namespace {
  tflite::ErrorReporter* error_reporter = nullptr;
  const tflite::Model* model = nullptr;
  tflite::MicroInterpreter* interpreter = nullptr;
  TfLiteTensor* model_input = nullptr;
  TfLiteTensor* model_output = nullptr;  //serve?
  //FeatureProvider* feature_provider = nullptr;
  //RecognizeCommands* recognizer = nullptr;
  int inference_count = 0; //serve?

  
  constexpr int kTensorArenaSize = 130*1024;
  // Keep aligned to 16 bytes for CMSIS
  alignas(16) uint8_t tensor_arena[kTensorArenaSize];
  //int8_t feature_buffer[kFeatureElementCount]; //messo nelle slide
  float* model_input_buffer = nullptr;
}  // namespace

Eloquent::ML::Port::SVM classifier;
// Eloquent::ML::Port::RandomForest classifier;

void setup() {
  // put your setup code here, to run once:
  //serial start
  Serial.begin(57600);
  while (!Serial)
    ;  //serial start failed

  //----------------------------- IMU setup -----------------------------//

  //Imu start check
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
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


  // print IMU starting status
  Serial.print("Accelerometer Full Scale = ±");
  Serial.print(IMU.getAccelFS());
  Serial.println("m/s^2");
  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.getAccelODR());
  Serial.println(" Hz \n");
  delay(1000);

  //Serial.println(" X \t Y \t Z ");

  //tflite::InitializeTarget(); NON LO TROVA E CREDO NON SERVA
  static tflite::MicroErrorReporter micro_error_reporter;
  error_reporter = &micro_error_reporter;
  // // Map the model into a usable data structure. This doesn't involve any
  // // copying or parsing, it's a very lightweight operation.
  model = tflite::GetModel(stride_model);
  if (model->version() != 3) { //impostato TFLITE_SCHEMA_VERSION = 3 preso dagli esempi
    TF_LITE_REPORT_ERROR(error_reporter,
                         "Model provided is schema version %d not equal "
                         "to supported version %d.",
                         model->version(), 3);
    return;
  }

  // This pulls in all the operation implementations we need.
  // NOLINTNEXTLINE(runtime-global-variables)
  static tflite::AllOpsResolver resolver; //poi possiamo farlo riga per riga per ottimizzare

  // Build an interpreter to run the model with.
  static tflite::MicroInterpreter static_interpreter(model, resolver, tensor_arena, kTensorArenaSize, error_reporter); //error_resolver non riesco a metterlo
  interpreter = &static_interpreter;

  // Allocate memory from the tensor_arena for the model's tensors.
  TfLiteStatus allocate_status = interpreter->AllocateTensors();
  if (allocate_status != kTfLiteOk) {
    TF_LITE_REPORT_ERROR(error_reporter, "AllocateTensors() failed");
    return;
  }

  // Obtain pointers to the model's input and output tensors.
  model_input = interpreter->input(0);
  //vedere se aggiungere if slide
  //model_input_buffer = reinterpret_cast<float*>(model_input->data.int8);
  model_input_buffer = model_input->data.f;
  model_output = interpreter->output(0);

  // Keep track of how many inferences we have performed.
  inference_count = 0;
}

void loop() {
  //define variables for IMU reads
  int16_t acc[3], gyro[3];

  // Dichiarazione delle variabili per l'accumulo dei campioni
  float data[500][6];
  int sample_count = 0;

  Serial.println("Inizio lettura dati...");
  // Accumula i campioni fino a raggiungere 500
  while (sample_count < 500) {
    if (IMU.accelAvailable() && IMU.gyroAvailable()) {
      IMU.readRawAccelArr(acc);
      IMU.readRawGyroArr(gyro);

      // Converti i valori letti nell'intervallo desiderato per il modello
      float accel_x = static_cast<float>(acc[0]);
      float accel_y = static_cast<float>(acc[1]);
      float accel_z = static_cast<float>(acc[2]);
      float gyro_x = static_cast<float>(gyro[0]);
      float gyro_y = static_cast<float>(gyro[1]);
      float gyro_z = static_cast<float>(gyro[2]);

      // Salva i campioni nell'array data
      data[sample_count][0] = accel_x;
      data[sample_count][1] = accel_y;
      data[sample_count][2] = accel_z;
      data[sample_count][3] = gyro_x;
      data[sample_count][4] = gyro_y;
      data[sample_count][5] = gyro_z;

      sample_count++;
    }
  }
  
  Serial.println("Normalizzazione...");
  // Calcola media e deviazione standard
  /*float media[6] = {0.0};
  float stdev[6] = {0.0};
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 500; j++) {
      media[i] += data[j][i];
    }
    media[i] /= 500.0;
  }

  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 500; j++) {
      stdev[i] += (data[j][i] - media[i]) * (data[j][i] - media[i]);
    }
    stdev[i] = sqrt(stdev[i] / 500.0);
  }

  // Normalizza i dati
  for (int i = 0; i < 500; i++) {
    for (int j = 0; j < 6; j++) {
      data[i][j] = (data[i][j] - media[j]) / stdev[j];  
    }
  }*/

  for (int i = 0; i < 500; i++) {
    for (int j = 0; j < 6; j++) {
      data[i][j] = data[i][j] / 16384;  
    }
  }

  //Prepara il tensore di input per l'inferenza
  int input_index = 0;
  for (int i = 0; i < 500; i++) {
    for (int j = 0; j < 6; j++) {
      model_input_buffer[input_index] = data[i][j];
      input_index++;
    }
  }

  // Copia i dati di input nell'array del tensore
  //memcpy(model_input_buffer, data, sizeof(float) * 500 * 6); //QUESTO NON FUNZIONA

  // //only to check
  // for (int i = 0; i < 500; i++) {
  //   for (int j = 0; j < 6; j++) {
  //     Serial.print(model_input_buffer[i * 6 + j],6);
  //     Serial.print("\t");
  //   }
  //   Serial.println();
  // }

  /*Serial.println();
  Serial.println();

  for (int i = 0; i < 500; i++) {
    for (int j = 0; j < 6; j++) {
      Serial.print(data[i][j],6);
      Serial.print("\t");
    }
    Serial.println();
  }*/

  Serial.println("Inizio inferenza...");
  unsigned long startTime = millis();
  
  // Perform inference
  TfLiteStatus invoke_status = interpreter->Invoke();
  if (invoke_status != kTfLiteOk) {
    TF_LITE_REPORT_ERROR(error_reporter, "Invoke failed");
    return;
  }


  // Leggere i risultati dal tensore di output
  float* output_data = model_output->data.f;
  int output_size = model_output->dims->data[1];  // Dimensione dell'output

   for (int i = 0; i < 256; i++) {
     Serial.println(output_data[i]);
  }

  // Trovare la classe predetta
  int predicted_class = classifier.predict(output_data);

  // Stampa la classe predetta
  Serial.print("Classe predetta: ");
  Serial.println(predicted_class);

  Serial.print("Tempo di inferenza: ");
  Serial.println(millis() - startTime);
}
