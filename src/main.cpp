#include <Arduino.h>
#include <Arduino_LSM9DS1.h>
#include <Arduino_LPS22HB.h>
#include <Arduino_HTS221.h>
#include <Arduino_APDS9960.h>

float convertG(float val)
{
  return 9.81 * val;
}

void setup()
{
  Serial.begin(9600);
  while (!Serial)
    ;
  Serial.println("Started");

  if (!IMU.begin())
  {
    Serial.println("Failed to initialia_ze IMU!");
    while (1)
      ;
  }

  if (!HTS.begin())
  {
    Serial.println("Failed to initialize humidity temperature sensor!");
    while (1)
      ;
  }
  if (!BARO.begin())
  {
    Serial.println("Failed to initialize pressure sensor!");
    while (1)
      ;
  }
  if (!APDS.begin())
  {
    Serial.println("Error initializing APDS9960 sensor.");
    while (1)
      ;
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in G's");
  Serial.println("a_x a_y a_z");

  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Gyroscope in degrees/second");
  Serial.println("G_x G_y G_z");

  Serial.print("Magnetic field sample rate = ");
  Serial.print(IMU.magneticFieldSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Magnetic Field in µT");
  Serial.println("M_X M_Y M_Z");
}

int proximity = 0;
int r = 0, g = 0, b = 0;
unsigned long lastUpdate = 0;

void loop()
{
  if (millis() - lastUpdate > 1000)
  {
    lastUpdate = millis();

    float a_x, a_y, a_z;

    if (IMU.accelerationAvailable())
    {
      IMU.readAcceleration(a_x, a_y, a_z);
      Serial.print("Accelerometer : ");
      Serial.print(convertG(a_x));
      Serial.print(' ');
      Serial.print(convertG(a_y));
      Serial.print(' ');
      Serial.print(convertG(a_z));
    }
    float G_x, G_y, G_z;

    if (IMU.gyroscopeAvailable())
    {
      IMU.readGyroscope(G_x, G_y, G_z);
      Serial.print(" ");
      Serial.print("Gyroscope : ");
      Serial.print(G_x);
      Serial.print(' ');
      Serial.print(G_y);
      Serial.print(' ');
      Serial.print(G_z);
    }

    float M_x, M_y, M_z;

    if (IMU.magneticFieldAvailable())
    {
      IMU.readMagneticField(M_x, M_y, M_z);
      Serial.print(' ');
      Serial.print("Magnetometer : ");
      Serial.print(M_x);
      Serial.print(' ');
      Serial.print(M_y);
      Serial.print(' ');
      Serial.print(M_z);
    }

    float temperature = HTS.readTemperature();
    float humidity = HTS.readHumidity();
    float pressure = BARO.readPressure();

    // print each of the sensor values
    Serial.print(' ');
    Serial.print("Temperature : ");
    Serial.print(temperature);
    // Serial.print(" °C");
    Serial.print(' ');
    Serial.print("Humidity : ");
    Serial.print(humidity);
    // Serial.println(" %");
    Serial.print(' ');
    Serial.print("Pressure : ");
    Serial.println(pressure*10);
    // Serial.println(" kPa");

    // Check if a proximity reading is available.
    if (APDS.proximityAvailable())
    {
      proximity = APDS.readProximity();
    }

    // check if a gesture reading is available
    if (APDS.gestureAvailable())
    {
      int gesture = APDS.readGesture();
      switch (gesture)
      {
      case GESTURE_UP:
        Serial.println("Detected UP gesture");
        break;

      case GESTURE_DOWN:
        Serial.println("Detected DOWN gesture");
        break;

      case GESTURE_LEFT:
        Serial.println("Detected LEFT gesture");
        break;

      case GESTURE_RIGHT:
        Serial.println("Detected RIGHT gesture");
        break;

      default:
        Serial.println("No gesture Detected ");
        break;
      }
    }

    if (APDS.colorAvailable())
    {
      APDS.readColor(r, g, b);
    }
    Serial.print("Proximity : ");
    Serial.print(proximity);
    Serial.print(" rgb : ");
    Serial.print(r);
    Serial.print(",");
    Serial.print(g);
    Serial.print(",");
    Serial.println(b);
  }
}