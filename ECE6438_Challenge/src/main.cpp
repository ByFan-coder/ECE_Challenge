#include <mbed.h>
#include <vector>
#include "drivers/LCD_DISCO_F429ZI.h"
#include "drivers/TS_DISCO_F429ZI.h"
#include <array>
#include <limits>
#include <cmath>
#include <math.h>

/*
    Group 73 ecegy-6384
    MembersWEIYI LUO, JUNJIE MAI, JIAKUN LIAN

    The code implements a gesture-based unlocking system using a gyroscope, 
    touch screen interface, and a dynamic time warping (DTW) algorithm for gesture matching.
    It initializes and calibrates the gyroscope via SPI communication to collect three-axis angular velocity data
    filters noise using thresholds, and stores recorded gestures in memory or Flash for persistence.
    The system features a user interface with an LCD screen and touch input
    allowing users to record gestures or attempt unlocking by matching their input with the stored gesture key
*/


// Register addresses
#define CTRL_REG_1 0x20 // control register 1
#define CTRL_REG_3 0x22 // control register 3
#define CTRL_REG_4 0x23 // control register 4

#define OUT_X_L 0x28 // X-axis angular rate data Low

#define INT1_SRC 0x31 // interrupt 1 source register

#define ODR_200_CUTOFF_50 0x60

// Interrupt configurations
#define INT2_DRDY 0x08 // Data ready on DRDY/INT2 pin

// Fullscale selections
#define FULL_SCALE_245 0x00      // full scale 245 dps
#define FULL_SCALE_500 0x10      // full scale 500 dps
#define FULL_SCALE_2000 0x20     // full scale 2000 dps
#define FULL_SCALE_2000_ALT 0x30 // full scale 2000 dps

// Sensitivities in dps/digit
#define SENSITIVITY_245 0.00875f // 245 dps typical sensitivity
#define SENSITIVITY_500 0.0175f  // 500 dps typical sensitivity
#define SENSITIVITY_2000 0.07f   // 2000 dps typical sensitivity

// Convert constants
#define MY_LEG 1              // put board on left leg 0.8m above ground
#define DEGREE_TO_RAD 0.0175f // rad = dgree * (pi / 180)

#define POWERON 0x0f  // turn gyroscope
#define POWEROFF 0x00 // turnoff gyroscope

/*
float euclidean_distance(const std::vector<float>& vec1, const std::vector<float>& vec2) {
    float sum = 0.0f;
    for (size_t i = 0; i < vec1.size(); i++) {
        sum += pow(vec1[i] - vec2[i], 2);
    }
    return sqrt(sum);
}
*/


// Initialization parameters
typedef struct
{
    uint8_t conf1;       // output data rate
    uint8_t conf3;       // interrupt configuration
    uint8_t conf4;       // full sacle selection
} Gyroscope_Init_Parameters;

// Raw data
typedef struct
{
    int16_t x_raw; // X-axis raw data
    int16_t y_raw; // Y-axis raw data
    int16_t z_raw; // Z-axis raw data
} Gyroscope_RawData;


//initialize the SPI interface for communication with GYRO
SPI gyroscope(PF_9, PF_8, PF_7); // mosi, miso, sclk
DigitalOut cs(PC_1);

int16_t x_threshold; // X-axis calibration threshold
int16_t y_threshold; // Y-axis calibration threshold
int16_t z_threshold; // Z-axis calibration threshold

int16_t x_sample; // X-axis zero-rate level sample
int16_t y_sample; // Y-axis zero-rate level sample
int16_t z_sample; // Z-axis zero-rate level sample

float sensitivity = 0.0f;

Gyroscope_RawData *gyro_raw;

// Write I/O
void WriteByte(uint8_t address, uint8_t data)
{
  cs = 0;
  gyroscope.write(address);
  gyroscope.write(data);
  cs = 1;
}

// Get raw data from gyroscope
void GetGyroValue(Gyroscope_RawData *rawdata)
{
  cs = 0;
  gyroscope.write(OUT_X_L | 0x80 | 0x40); // auto-incremented read
  rawdata->x_raw = gyroscope.write(0xff) | gyroscope.write(0xff) << 8;
  rawdata->y_raw = gyroscope.write(0xff) | gyroscope.write(0xff) << 8;
  rawdata->z_raw = gyroscope.write(0xff) | gyroscope.write(0xff) << 8;
  cs = 1;
}

// Calibrate gyroscope before recording
void CalibrateGyroscope(Gyroscope_RawData *rawdata)
{
  int16_t sumX = 0;
  int16_t sumY = 0;
  int16_t sumZ = 0;
  for (int i = 0; i < 128; i++)
  {
    GetGyroValue(rawdata);
    sumX += rawdata->x_raw;
    sumY += rawdata->y_raw;
    sumZ += rawdata->z_raw;
    x_threshold = max(x_threshold, rawdata->x_raw);
    y_threshold = max(y_threshold, rawdata->y_raw);
    z_threshold = max(z_threshold, rawdata->z_raw);
    wait_us(10000);
  }

  x_sample = sumX >> 7; // 128 is 2^7
  y_sample = sumY >> 7;
  z_sample = sumZ >> 7;
}

// Initiate gyroscope, set up control registers
void InitiateGyroscope(Gyroscope_Init_Parameters *init_parameters, Gyroscope_RawData *init_raw_data)
{
  gyro_raw = init_raw_data;
  cs = 1;
  // set up gyroscope
  gyroscope.format(8, 3);       // 8 bits per SPI frame; polarity 1, phase 0
  gyroscope.frequency(1000000); // clock frequency deafult 1 MHz max:10MHz

  WriteByte(CTRL_REG_1, init_parameters->conf1 | POWERON); // set ODR Bandwidth and enable all 3 axises
  WriteByte(CTRL_REG_3, init_parameters->conf3);           // DRDY enable
  WriteByte(CTRL_REG_4, init_parameters->conf4);           // LSB, full sacle selection: 500dps

  switch (init_parameters->conf4)
  {
  case FULL_SCALE_245:
    sensitivity = SENSITIVITY_245;
    break;

  case FULL_SCALE_500:
    sensitivity = SENSITIVITY_500;
    break;

  case FULL_SCALE_2000:
    sensitivity = SENSITIVITY_2000;
    break;

  case FULL_SCALE_2000_ALT:
    sensitivity = SENSITIVITY_2000;
    break;
  }

  CalibrateGyroscope(gyro_raw); // calibrate the gyroscope and find the threshold for x, y, and z.
}

// convert raw data to dps
float ConvertToDPS(int16_t axis_data)
{
  float dps = axis_data * sensitivity;
  return dps;
}

// convert raw data to calibrated data directly
void GetCalibratedRawData()
{
  GetGyroValue(gyro_raw);

  // offset the zero rate level
  gyro_raw->x_raw -= x_sample;
  gyro_raw->y_raw -= y_sample;
  gyro_raw->z_raw -= z_sample;

  // put data below threshold to zero
  if (abs(gyro_raw->x_raw) < abs(x_threshold))
    gyro_raw->x_raw = 0;
  if (abs(gyro_raw->y_raw) < abs(y_threshold))
    gyro_raw->y_raw = 0;
  if (abs(gyro_raw->z_raw) < abs(z_threshold))
    gyro_raw->z_raw = 0;
}

vector<array<float, 3>> gesture_key; // the gesture key
vector<array<float, 3>> unlocking_record; // the unlocking record

int err = 0; // for error checking

// tollerance
#define EPSILON 6000 //2000-6000

//Text size
#define TEXT_SIZE 24

// Events
#define IS_GUESTURE_SAVED 1
#define IS_OPEN 2
#define DELETE 4
#define READY 8
#define USER_BUTTON PA_0

InterruptIn gyro_int2(PA_2, PullDown);
InterruptIn ResetButton(USER_BUTTON, PullDown);

DigitalOut green_led(LED1);
DigitalOut red_led(LED2);

// LCD object
LCD_DISCO_F429ZI lcd; 

// Touch screen object
TS_DISCO_F429ZI ts; 

EventFlags flags; // Event flags

Timer timer; // Timer

const int Button1X = 50; //button for screen 10
const int Button1Y = 110;
const int Button1Width = 150; //100
const int Button1Height = 70; //50
const char *Button1Label = "UNLOCK";

const int Button2X = 50; //button for screen 121
const int Button2Y = 190; //100
const int Button2Width = 150; //100
const int Button2Height = 70; //50
const char *Button2Label = "RECORD";

void button_press() // button press ISR
{
    flags.set(DELETE);
}
void onGyroDataReady() // Gyrscope data ready ISR
{
    flags.set(READY);
}

const int message_x = 5;
const int message_y = 30;
const char *message = "ECEGY 6483";
const int text_x = 5;
const int text_y = 60;
const char *text_0 = "NO GESTURE RECORDED";
const char *text_1 = "LOCKED";

bool detect_touch(int touch_x, int touch_y, int button_x, int button_y, int button_width, int button_height) {
    return (touch_x >= button_x && touch_x <= button_x + button_width &&
            touch_y >= button_y && touch_y <= button_y + button_height);
}

// Display the buttons on the LCD
void display()
{
    TS_StateTypeDef ts_state;

    if (ts.Init(lcd.GetXSize(), lcd.GetYSize()) != TS_OK)
    {
        printf("Failed to initialize the touch screen!\r\n");
        return;
    }

    // initialize a string display_buffer that can be draw on the LCD to dispaly the status
    char display_buffer[50];

    while (1)
    {
        ts.GetState(&ts_state);
        if (ts_state.TouchDetected)
        {
            int touch_x = ts_state.X;
            int touch_y = 306 - ts_state.Y; //256 height for whole screen
            printf("Touch values: x = %d, y = %d\n", touch_x, touch_y);
            // Check if the touch is inside record button
            if (detect_touch(touch_x, touch_y, Button2X, Button2Y, Button2Width, Button2Height))
            {
                sprintf(display_buffer, "Recording");
                lcd.SetTextColor(LCD_COLOR_BLACK);                  
                lcd.FillRect(0, text_y, lcd.GetXSize(), TEXT_SIZE); 
                lcd.SetTextColor(LCD_COLOR_RED);                   
                lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);
                ThisThread::sleep_for(1s);
                flags.set(IS_GUESTURE_SAVED);
            }

            // Check if the touch is inside unlock button
            if (detect_touch(touch_x, touch_y, Button1X, Button1Y, Button1Width, Button1Height))
            {
                sprintf(display_buffer, "Wait");
                lcd.SetTextColor(LCD_COLOR_BLACK);                  
                lcd.FillRect(0, text_y, lcd.GetXSize(), TEXT_SIZE); 
                lcd.SetTextColor(LCD_COLOR_RED);                   
                lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);
                ThisThread::sleep_for(1s);
                flags.set(IS_OPEN);
            }
        }
        ThisThread::sleep_for(10ms);
    }
}

bool store_gyro_Data(vector<array<float, 3>> &gesture, uint32_t address)
{
    FlashIAP flash;
    flash.init();

    // Calculate the total size of the data to be stored in bytes
    uint32_t data_size = gesture.size() * sizeof(array<float, 3>);

    // Erase the flash sector
    flash.erase(address, data_size);

    // Write the data to flash
    int write_result = flash.program(gesture.data(), address, data_size);

    flash.deinit();

    return write_result == 0;
}

vector<array<float, 3>> read_gyro_data(uint32_t address, size_t length)
{
    vector<array<float, 3>> gesture_key(length);

    FlashIAP flash;
    flash.init();

    // Read the data from flash
    flash.read(gesture_key.data(), address, length * sizeof(array<float, 3>));

    flash.deinit();

    return gesture_key;
}

// create a button on the LCD
void create_button(int x, int y, int width, int height, const char *label)
{
    lcd.SetTextColor(LCD_COLOR_LIGHTBLUE);
    lcd.FillRect(x, y, width, height);
    lcd.DisplayStringAt(x + width / 2 - strlen(label) * 19, y + height / 2 - 8, (uint8_t *)label, CENTER_MODE);
}

// Process the data from the gyroscope
void process_gyro_data(vector<array<float, 3>> &data)
{
    float threshold = 0.00001;
    auto ptr = data.begin();
    while (abs((*ptr)[0]) <= threshold && abs((*ptr)[1]) <= threshold && abs((*ptr)[2]) <= threshold)
    {
        ptr++;
    }
    if (ptr == data.end())
        return;      // all data less than threshold
    auto lptr = ptr; // record the left bound
    // start searching from end to front
    ptr = data.end() - 1;
    while (abs((*ptr)[0]) <= threshold && abs((*ptr)[1]) <= threshold && abs((*ptr)[2]) <= threshold)
    {
        ptr--;
    }
    auto rptr = ptr; // record the right bound
    // start moving elements to the front
    auto replace_ptr = data.begin();
    for (; replace_ptr != lptr && lptr <= rptr; replace_ptr++, lptr++)
    {
        *replace_ptr = *lptr;
    }
    // trim the end
    if (lptr > rptr)
    {
        data.erase(replace_ptr, data.end());
    }
    else
    {
        data.erase(rptr + 1, data.end());
    }
}

// DTW calculates an optimal match between two given sequences
float dtw(const std::vector<std::array<float, 3>>& seq1, const std::vector<std::array<float, 3>>& seq2) {
    size_t n = seq1.size();
    size_t m = seq2.size();
    std::vector<std::vector<float>> cost_matrix(n, std::vector<float>(m, std::numeric_limits<float>::infinity()));

    auto euclidean_distance = [](const std::array<float, 3>& a, const std::array<float, 3>& b) {
        return sqrt(pow(a[0] - b[0], 2) + pow(a[1] - b[1], 2) + pow(a[2] - b[2], 2));
    };

    cost_matrix[0][0] = euclidean_distance(seq1[0], seq2[0]);

    for (size_t i = 1; i < n; ++i) {
        cost_matrix[i][0] = cost_matrix[i - 1][0] + euclidean_distance(seq1[i], seq2[0]);
    }
    for (size_t j = 1; j < m; ++j) {
        cost_matrix[0][j] = cost_matrix[0][j - 1] + euclidean_distance(seq1[0], seq2[j]);
    }
    for (size_t i = 1; i < n; ++i) {
        for (size_t j = 1; j < m; ++j) {
            float cost = euclidean_distance(seq1[i], seq2[j]);
            cost_matrix[i][j] = cost + std::min({
                cost_matrix[i - 1][j],    // top
                cost_matrix[i][j - 1],    // left
                cost_matrix[i - 1][j - 1] // diagonal
            });
        }
    }

    return cost_matrix[n - 1][m - 1];
}

// record the gesture key
void record_gyro()
{

    // intiiate the gyroscope
    Gyroscope_Init_Parameters params;
    params.conf1 = ODR_200_CUTOFF_50;
    params.conf3 = INT2_DRDY;
    params.conf4 = FULL_SCALE_500;

    // Set up gyroscope's raw data
    Gyroscope_RawData raw_data;

    // initialize LCD display buffer
    char display_buffer[50];

    // check the signal and set the flag
    // for the first sample.
    if (!(flags.get() & READY) && (gyro_int2.read() == 1))
    {
        flags.set(READY);
    }

    while (1)
    {
        vector<array<float, 3>> temp_key; 

        auto flag_check = flags.wait_any(IS_GUESTURE_SAVED | IS_OPEN | DELETE);

        if (flag_check & DELETE)
        {
            // Erase the gesture key
            sprintf(display_buffer, "Erasing");
            lcd.SetTextColor(LCD_COLOR_BLACK);                  
            lcd.FillRect(0, text_y, lcd.GetXSize(), TEXT_SIZE); 
            lcd.SetTextColor(LCD_COLOR_RED);                  
            lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);
            gesture_key.clear();
            
            // Erase the unlocking record
            lcd.SetTextColor(LCD_COLOR_BLACK);                  
            lcd.FillRect(0, text_y, lcd.GetXSize(), TEXT_SIZE); 
            lcd.SetTextColor(LCD_COLOR_RED);                   
            lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);
            unlocking_record.clear();

            // Reset the LED and print the message
            green_led = 1;
            red_led = 0;
            sprintf(display_buffer, "All Erasing finish.");
            lcd.SetTextColor(LCD_COLOR_BLACK);                  
            lcd.FillRect(0, text_y, lcd.GetXSize(), TEXT_SIZE); 
            lcd.SetTextColor(LCD_COLOR_RED);                  
            lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);
        }

        if (flag_check & (IS_GUESTURE_SAVED | IS_OPEN))
        {
            sprintf(display_buffer, "Wait");
            lcd.SetTextColor(LCD_COLOR_BLACK);                  
            lcd.FillRect(0, text_y, lcd.GetXSize(), TEXT_SIZE); 
            lcd.SetTextColor(LCD_COLOR_RED);                   
            lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);

            ThisThread::sleep_for(1s);

            lcd.SetTextColor(LCD_COLOR_BLACK);                  
            lcd.FillRect(0, text_y, lcd.GetXSize(), TEXT_SIZE); 
            lcd.SetTextColor(LCD_COLOR_RED);                   
            lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);

            // Initiate gyroscope
            InitiateGyroscope(&params, &raw_data);

            // start recording gesture
            lcd.SetTextColor(LCD_COLOR_BLACK);                  
            lcd.FillRect(0, text_y, lcd.GetXSize(), TEXT_SIZE);
            lcd.SetTextColor(LCD_COLOR_RED);                   
            lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);
            ThisThread::sleep_for(1s);
            
            lcd.SetTextColor(LCD_COLOR_BLACK);                  
            lcd.FillRect(0, text_y, lcd.GetXSize(), TEXT_SIZE); 
            lcd.SetTextColor(LCD_COLOR_RED);                   
            lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);
            ThisThread::sleep_for(1s);

            sprintf(display_buffer, "Recording in 1sec ");

            lcd.SetTextColor(LCD_COLOR_BLACK);                  
            lcd.FillRect(0, text_y, lcd.GetXSize(), TEXT_SIZE); 
            lcd.SetTextColor(LCD_COLOR_RED);                  
            lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);
            ThisThread::sleep_for(1s);

            sprintf(display_buffer, "Recording");
            lcd.SetTextColor(LCD_COLOR_BLACK);                  
            lcd.FillRect(0, text_y, lcd.GetXSize(), TEXT_SIZE); 
            lcd.SetTextColor(LCD_COLOR_RED);                   
            lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);
            
            // gyro data recording loop
            timer.start();
            while (timer.elapsed_time() < 5s)
            {
                // Wait for the gyroscope data to be ready
                flags.wait_all(READY);
                // Read the data from the gyroscope
                GetCalibratedRawData();
                // Add the converted data to the gesture_key vector
                temp_key.push_back({ConvertToDPS(raw_data.x_raw), ConvertToDPS(raw_data.y_raw), ConvertToDPS(raw_data.z_raw)});
                ThisThread::sleep_for(50ms); // 20Hz
            }
            timer.stop();  // Stop timer
            timer.reset(); // Reset timer

            // trim zeros
            process_gyro_data(temp_key);

            sprintf(display_buffer, "Done");
            lcd.SetTextColor(LCD_COLOR_BLACK);                  
            lcd.FillRect(0, text_y, lcd.GetXSize(), TEXT_SIZE); 
            lcd.SetTextColor(LCD_COLOR_RED);                   
            lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);
        }

        // check the flag see if it is recording or unlocking
        if (flag_check & IS_GUESTURE_SAVED)
        {
            if (gesture_key.empty())
            {
                lcd.SetTextColor(LCD_COLOR_BLACK);                  
                lcd.FillRect(0, text_y, lcd.GetXSize(), TEXT_SIZE); 
                lcd.SetTextColor(LCD_COLOR_RED);                   
                lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);

                // save the key
                gesture_key = temp_key;

                // clear temp_key
                temp_key.clear();

                // toggle led
                red_led = 1;
                green_led = 0;

                sprintf(display_buffer, "Gesture saved");
                lcd.SetTextColor(LCD_COLOR_BLACK);                  
                lcd.FillRect(0, text_y, lcd.GetXSize(), TEXT_SIZE); 
                lcd.SetTextColor(LCD_COLOR_RED);                   
                lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);
            }
            else
            {
                lcd.SetTextColor(LCD_COLOR_BLACK);                  
                lcd.FillRect(0, text_y, lcd.GetXSize(), TEXT_SIZE); 
                lcd.SetTextColor(LCD_COLOR_RED);                   
                lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);

                ThisThread::sleep_for(1s);
                
                // clear old key
                gesture_key.clear();

                // save new key
                gesture_key = temp_key;

                sprintf(display_buffer, "New gesture saved.");
                lcd.SetTextColor(LCD_COLOR_BLACK);                  
                lcd.FillRect(0, text_y, lcd.GetXSize(), TEXT_SIZE); 
                lcd.SetTextColor(LCD_COLOR_RED);                   
                lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);

                // clear temp_key
                temp_key.clear();

                // toggle led
                red_led = 1;
                green_led = 0;
            }
        }
        else if (flag_check & IS_OPEN)
        {
            flags.clear(IS_OPEN);
            sprintf(display_buffer, "Wait");
            lcd.SetTextColor(LCD_COLOR_BLACK);                  
            lcd.FillRect(0, text_y, lcd.GetXSize(), TEXT_SIZE); 
            lcd.SetTextColor(LCD_COLOR_RED);                   
            lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);

            unlocking_record = temp_key; // save the unlocking record
            temp_key.clear(); // clear temp_key

            // check if the gesture key is empty
            if (gesture_key.empty())
            {
                sprintf(display_buffer, "NO GESTURE SAVED.");
                lcd.SetTextColor(LCD_COLOR_BLACK);                  
                lcd.FillRect(0, text_y, lcd.GetXSize(), TEXT_SIZE); 
                lcd.SetTextColor(LCD_COLOR_RED);                   
                lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);

                unlocking_record.clear(); // clear unlocking record

                // toggle led
                green_led = 1;
                red_led = 0;
            }
            else // compare the unlocking record with the gesture key
            {
                
                int unlock = 0; // counter for the coordinates that are above threshold

                float distance = dtw(gesture_key, unlocking_record); 

                if (err != 0)
                {
                    printf("Error in distance: different sizes!\n");
                }
                else
                {
                    printf("Distance: %f \n", distance);
                    if (distance < EPSILON) //wait for discu
                    {
                        unlock=1;
                    }
                    printf("%d \n", unlock);
                }

                if (unlock==1) // need to find a better threshold
                {
                    sprintf(display_buffer, "UNLOCK SUCESSFULLY");
                    lcd.SetTextColor(LCD_COLOR_BLACK);                  
                    lcd.FillRect(0, text_y, lcd.GetXSize(), TEXT_SIZE); 
                    lcd.SetTextColor(LCD_COLOR_RED);                   
                    lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);
                    
                    // toggle led
                    green_led = 1;
                    red_led = 0;

                    // clear unlocking record
                    unlocking_record.clear();
                    unlock = 0;
                }
                else
                {
                    sprintf(display_buffer, " Fail ");
                    lcd.SetTextColor(LCD_COLOR_BLACK);                  
                    lcd.FillRect(0, text_y, lcd.GetXSize(), TEXT_SIZE); 
                    lcd.SetTextColor(LCD_COLOR_RED);                   
                    lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);

                    // toggle led
                    green_led = 0;
                    red_led = 1;

                    // clear unlocking record
                    unlocking_record.clear();
                    unlock = 0;
                }
            }
        }
        ThisThread::sleep_for(100ms);
    }
}


int main()
{
    lcd.Clear(LCD_COLOR_GRAY);
    // Draw buttons on the screen
create_button(Button1X, Button1Y, Button1Width, Button1Height, Button1Label);
create_button(Button2X, Button2Y, Button2Width, Button2Height, Button2Label);

// Display a message on the LCD screen
lcd.DisplayStringAt(message_x, message_y, (uint8_t *)message, CENTER_MODE);

// Initialize interrupts for user button and gyroscope
ResetButton.rise(&button_press);
gyro_int2.rise(&onGyroDataReady);

// Check if the gesture key is empty
if (gesture_key.empty())
{
    // Turn on the green LED and display a message on the LCD screen
    red_led = 0;
    green_led = 1;
    lcd.DisplayStringAt(text_x, text_y, (uint8_t *)text_0, CENTER_MODE);
}
else
{
    // Turn on the red LED and display a message on the LCD screen
    red_led = 1;
    green_led = 0;
    lcd.DisplayStringAt(text_x, text_y, (uint8_t *)text_1, CENTER_MODE);
}

// Initialize gyroscope
Thread key_saving;
key_saving.start(callback(record_gyro));

// Initialize touch screen
Thread touch_thread;
touch_thread.start(callback(display));

// Run the main loop
while (1)
{
    // Sleep for 100ms before repeating the loop
    ThisThread::sleep_for(100ms);
}
}