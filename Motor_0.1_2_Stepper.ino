#include <AccelStepper.h>

// Pin setup untuk Motor 1
#define stepPin1 9
#define dirPin1 8
#define enablePin1 13

// Pin setup untuk Motor 2
#define stepPin2 3
#define dirPin2 2
#define enablePin2 7

// Konfigurasi langkah motor
const int stepsPerRevolution1 = 200;  // Langkah per putaran penuh motor 1
const int stepsPerRevolution2 = 100;  // Langkah per putaran penuh motor 2         
const int teethCount = 20;            // Jumlah gerigi roda
const float pitch = 0.2;              // Jarak antar gerigi dalam cm
const float circumference = teethCount * pitch;  // Keliling roda (dalam cm)
const float distancePerStep1 = circumference /stepsPerRevolution1;  // Jarak per langkah motor 1
const float distancePerStep2 = circumference /stepsPerRevolution2;  // Jarak per langkah motor 2

long currentPositionMotor1 = 0;  // Posisi saat ini (dalam langkah) motor 1
long currentPositionMotor2 = 0;  // Posisi saat ini (dalam langkah) motor 2

// Membuat objek AccelStepper untuk kedua motor
AccelStepper motor1(AccelStepper::DRIVER, stepPin1, dirPin1);
AccelStepper motor2(AccelStepper::DRIVER, stepPin2, dirPin2);

void setup() {
  // Motor 1
  pinMode(enablePin1, OUTPUT);
  digitalWrite(enablePin1, LOW);  // Aktifkan driver untuk motor 1

  // Motor 2
  pinMode(enablePin2, OUTPUT);
  digitalWrite(enablePin2, LOW);  // Aktifkan driver untuk motor 2

  // Set kecepatan dan akselerasi untuk kedua motor
  motor1.setMaxSpeed(1000);  // Kecepatan maksimal motor 1
  motor1.setAcceleration(500);  // Akselerasi motor 1

  motor2.setMaxSpeed(1000);  // Kecepatan maksimal motor 2
  motor2.setAcceleration(500);  // Akselerasi motor 2

  Serial.begin(9600);
  Serial.println("Masukkan jarak untuk motor 1 dan motor 2 (format: jarak1,jarak2):");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');  // Baca input sebagai string
    input.trim();  // Hilangkan spasi atau karakter baru

    // Pisahkan input menjadi dua nilai
    int separatorIndex = input.indexOf(',');
    if (separatorIndex != -1) {
      // Ambil jarak untuk motor 1 dan motor 2
      float distanceMotor1 = input.substring(0, separatorIndex).toFloat();
      float distanceMotor2 = input.substring(separatorIndex + 1).toFloat();

      // Hitung jumlah langkah berdasarkan jarak yang ingin ditempuh
      long stepsMotor1 = round(distanceMotor1 / distancePerStep1);
      long stepsMotor2 = round(distanceMotor2 / distancePerStep2);

      Serial.print("Jarak Motor 1:");
      Serial.print(distanceMotor1);
     
      Serial.print("Jarak Motor 2: ");
      Serial.print(distanceMotor2);
    
      // Set tujuan motor untuk bergerak
      motor1.moveTo(stepsMotor1);
      motor2.moveTo(stepsMotor2);

      // Gerakkan kedua motor secara simultan
      while (motor1.distanceToGo() != 0 || motor2.distanceToGo() != 0) {
        motor1.run();
        motor2.run();
      }

  delay(5000);

    // Setelah motor mencapai tujuan, kembali ke posisi awal
    motor1.moveTo(0);  // Arahkan motor 1 kembali ke posisi 0
    motor2.moveTo(0);  // Arahkan motor 2 kembali ke posisi 0

    // Gerakkan kedua motor kembali ke posisi 0 secara simultan
    while (motor1.distanceToGo() != 0 || motor2.distanceToGo() != 0) {
    motor1.run();
    motor2.run();
}

Serial.println("Motor telah kembali ke posisi awal.");

    } else {
      Serial.println("Input tidak valid. Gunakan format: jarak1,jarak2");
    }
  }
}
