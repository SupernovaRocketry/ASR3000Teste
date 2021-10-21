#include <Arduino.h>   //Arduino
#include <Wire.h>      //I2C
#include <TinyGPS++.h> //GPS
#include <BMP280.h>    //BMP280
#include <SPI.h>       //SPI (LoRa e MicroSD)
#include <LoRa.h>      //LoRa
#include <FS.h>        //SD (File System)
#include <SD.h>        //SD


//Comentário Teste

const int MPU = 0x68;                  //Endereço I2C MPU
int AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ; //Variaveis MPU

int botao = 0; //Toggle Push Button

#define P0 1013.25 //Pressão 0 (BMP)
BMP280 bmp;        //Objeto BMP

static const int RXPin = 16, TXPin = 17; //Pinos RX/TX para GPS
static const uint32_t GPSBaud = 4800;    //Baudrate GPS
TinyGPSPlus gps;                         //Objeto GPS

const int csPin = 5;      // Chip Select (Slave Select do protocolo SPI) do modulo Lora
const int resetPin = 2;   // Reset do modulo LoRa
const int irqPin = 4;     // Pino DI0 LoRa
String outgoing;          // Mensagem de saída LoRa
byte localAddress = 0xBB; // Endereco deste dispositivo LoRa
byte msgCount = 0;        // Contador de mensagens enviadas
byte destination = 0xFF;  // Endereco do dispositivo para enviar a mensagem (0xFF envia para todos devices)
long lastSendTime = 0;    // TimeStamp da ultima mensagem enviada
int interval = 5000;      // Intervalo em ms no envio das mensagens (inicial 5s)

#define SCK 14  //CLK HSPI (SD)
#define MISO 12 //MISO HSPI (SD)
#define MOSI 13 //MOSI HSPI (SD)
#define CS 15   //CS HSPI (SD)

void listDir(fs::FS &fs, const char *dirname, uint8_t levels) //Lista Diretorio(SD)
{
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if (!root)
  {
    Serial.println("Failed to open directory");
    return;
  }
  if (!root.isDirectory())
  {
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file)
  {
    if (file.isDirectory())
    {
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if (levels)
      {
        listDir(fs, file.name(), levels - 1);
      }
    }
    else
    {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

void createDir(fs::FS &fs, const char *path) //Cria Diretorio (SD)
{
  Serial.printf("Creating Dir: %s\n", path);
  if (fs.mkdir(path))
  {
    Serial.println("Dir created");
  }
  else
  {
    Serial.println("mkdir failed");
  }
}

void removeDir(fs::FS &fs, const char *path) //Remove Diretorio (SD)
{
  Serial.printf("Removing Dir: %s\n", path);
  if (fs.rmdir(path))
  {
    Serial.println("Dir removed");
  }
  else
  {
    Serial.println("rmdir failed");
  }
}

void readFile(fs::FS &fs, const char *path) //Le arquivo (SD)
{
  Serial.printf("Reading file: %s\n", path);

  File file = fs.open(path);
  if (!file)
  {
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.print("Read from file: ");
  while (file.available())
  {
    Serial.write(file.read());
  }
  file.close();
}

void writeFile(fs::FS &fs, const char *path, const char *message) //Escreve Arquivo (SD)
{
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file)
  {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message))
  {
    Serial.println("File written");
  }
  else
  {
    Serial.println("Write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char *path, const char *message) //Adiciona a arquivo (SD)
{
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file)
  {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message))
  {
    Serial.println("Message appended");
  }
  else
  {
    Serial.println("Append failed");
  }
  file.close();
}

void renameFile(fs::FS &fs, const char *path1, const char *path2) //Renomeia arquivo (SD)
{
  Serial.printf("Renaming file %s to %s\n", path1, path2);
  if (fs.rename(path1, path2))
  {
    Serial.println("File renamed");
  }
  else
  {
    Serial.println("Rename failed");
  }
}

void deleteFile(fs::FS &fs, const char *path) //Apaga arquivo (SD)
{
  Serial.printf("Deleting file: %s\n", path);
  if (fs.remove(path))
  {
    Serial.println("File deleted");
  }
  else
  {
    Serial.println("Delete failed");
  }
}

void testFileIO(fs::FS &fs, const char *path) //Testa Escrita e Leitura (SD)
{
  File file = fs.open(path);
  static uint8_t buf[512];
  size_t len = 0;
  uint32_t start = millis();
  uint32_t end = start;
  if (file)
  {
    len = file.size();
    size_t flen = len;
    start = millis();
    while (len)
    {
      size_t toRead = len;
      if (toRead > 512)
      {
        toRead = 512;
      }
      file.read(buf, toRead);
      len -= toRead;
    }
    end = millis() - start;
    Serial.printf("%u bytes read for %u ms\n", flen, end);
    file.close();
  }
  else
  {
    Serial.println("Failed to open file for reading");
  }

  file = fs.open(path, FILE_WRITE);
  if (!file)
  {
    Serial.println("Failed to open file for writing");
    return;
  }

  size_t i;
  start = millis();
  for (i = 0; i < 2048; i++)
  {
    file.write(buf, 512);
  }
  end = millis() - start;
  Serial.printf("%u bytes written for %u ms\n", 2048 * 512, end);
  file.close();
}

void sendMessage(String outgoing) //Envia mensagem LoRa
{
  LoRa.beginPacket();            // Inicia o pacote da mensagem
  LoRa.write(destination);       // Adiciona o endereco de destino
  LoRa.write(localAddress);      // Adiciona o endereco do remetente
  LoRa.write(msgCount);          // Contador da mensagem
  LoRa.write(outgoing.length()); // Tamanho da mensagem em bytes
  LoRa.print(outgoing);          // Vetor da mensagem
  LoRa.endPacket();              // Finaliza o pacote e envia
  msgCount++;                    // Contador do numero de mensagnes enviadas
}

void onReceive(int packetSize) //Recebe Mensagem LoRa
{
  if (packetSize == 0)
    return; // Se nenhuma mesnagem foi recebida, retorna nada

  // Leu um pacote, vamos decodificar?
  int recipient = LoRa.read();       // Endereco de quem ta recebendo
  byte sender = LoRa.read();         // Endereco do remetente
  byte incomingMsgId = LoRa.read();  // Mensagem
  byte incomingLength = LoRa.read(); // Tamanho da mensagem

  String incoming = "";

  while (LoRa.available())
  {
    incoming += (char)LoRa.read();
  }

  if (incomingLength != incoming.length())
  {
    // check length for error
    Serial.println("erro!: o tamanho da mensagem nao condiz com o conteudo!");
    return;
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != 0xFF)
  {
    Serial.println("This message is not for me.");
    return; // skip rest of function
  }

  // Caso a mensagem seja para este dispositivo, imprime os detalhes
  Serial.println("Recebido do dispositivo: 0x" + String(sender, HEX));
  Serial.println("Enviado para: 0x" + String(recipient, HEX));
  Serial.println("ID da mensagem: " + String(incomingMsgId));
  Serial.println("Tamanho da mensagem: " + String(incomingLength));
  Serial.println("Mensagem: " + incoming);
  Serial.println("RSSI: " + String(LoRa.packetRssi()));
  Serial.println("Snr: " + String(LoRa.packetSnr()));
  Serial.println();
}

void displayInfo() //Mostra Dados GPS
{
  Serial.print(F("Location: "));
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10)
      Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10)
      Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10)
      Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10)
      Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}

void setup() //Inicialização ESP
{
  pinMode(32, OUTPUT); //Pino Buzzer
  pinMode(33, OUTPUT); //Pino LED
  pinMode(25, OUTPUT); //Pino OPTO1
  pinMode(26, OUTPUT); //Pino OPTO2
  pinMode(34, INPUT);  //Pino Push Button

  Serial.begin(115200); //Inicia Serial

  Wire.begin(); //Inicia I2C

  Wire.beginTransmission(MPU); //Inicia MPU
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  if (!bmp.begin()) //Inicia BMP
  {
    Serial.println("BMP init failed!");
    while (1)
      ;
  }
  else
    Serial.println("BMP init success!");
  bmp.setOversampling(4);

  Serial2.begin(9600, SERIAL_8N1, RXPin, TXPin); //Inicializa Interface GPS

  Serial.println(" Comunicacao LoRa Duplex - Ping&Pong ");
  LoRa.setPins(csPin, resetPin, irqPin); //Define pinos LoRa
  if (!LoRa.begin(915E6))                //Inicializa LoRa em 915MHz e verifica
  {
    Serial.println(" Erro ao iniciar modulo LoRa. Verifique a coenxao dos seus pinos!! ");
    while (true)
      ;
  }
  Serial.println(" Modulo LoRa iniciado com sucesso!!! :) ");

  SPIClass spi = SPIClass(HSPI);  //Classe HSPI (SD)
  spi.begin(SCK, MISO, MOSI, CS); //Inicializa HSPI (SD)

  if (!SD.begin(CS, spi, 80000000)) //Inicializa SD
  {
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType(); //Tipo Cartão SD
  if (cardType == CARD_NONE)
  {
    Serial.println("No SD card attached");
    return;
  }

  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC)
  {
    Serial.println("MMC");
  }
  else if (cardType == CARD_SD)
  {
    Serial.println("SDSC");
  }
  else if (cardType == CARD_SDHC)
  {
    Serial.println("SDHC");
  }
  else
  {
    Serial.println("UNKNOWN");
  }
  uint64_t cardSize = SD.cardSize() / (1024 * 1024); //Tamanho Cartão SD
  Serial.printf("SD Card Size: %lluMB\n", cardSize);

  listDir(SD, "/", 0);                                                     //Lista Diretorios
  createDir(SD, "/mydir");                                                 //Cria Diretorio
  listDir(SD, "/", 0);                                                     //Lista Diretorios
  removeDir(SD, "/mydir");                                                 //Remove Diretorio
  listDir(SD, "/", 2);                                                     //Lista Diretorios
  writeFile(SD, "/hello.txt", "Bora ");                                    //Escreve arquivo hello.txt
  appendFile(SD, "/hello.txt", "SUPERNOVA!");                              //Adiciona a arquivo hello.txt
  readFile(SD, "/hello.txt");                                              //Lê hello.txt
  deleteFile(SD, "/foo.txt");                                              //Apaga hello.txt
  renameFile(SD, "/hello.txt", "/foo.txt");                                //Renomeia hello.txt para foo.txt
  readFile(SD, "/foo.txt");                                                //Le foo.txt
  testFileIO(SD, "/test.txt");                                             //Testa Leitura e Escrita
  Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024)); //Espaco total
  Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));   //Espaco usado
}

void loop() //Loop ESP
{
  Wire.beginTransmission(MPU); //Comeca transmissao MPU
  Wire.write(0x3B);            //Starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true); //Solicita os dados do MPU

  //Armazena o valor dos sensores nas variaveis correspondentes
  AcX = Wire.read() << 8 | Wire.read(); //0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); //0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); //0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read() << 8 | Wire.read(); //0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = Wire.read() << 8 | Wire.read(); //0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); //0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); //0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  //Escreve valores MPU
  Serial.print("AcX = ");
  Serial.print(AcX);
  Serial.print(" | AcY = ");
  Serial.print(AcY);
  Serial.print(" | AcZ = ");
  Serial.print(AcZ);
  Serial.print(" | Tmp = ");
  Serial.print(Tmp / 340.00 + 36.53);
  Serial.print(" | GyX = ");
  Serial.print(GyX);
  Serial.print(" | GyY = ");
  Serial.print(GyY);
  Serial.print(" | GyZ = ");
  Serial.println(GyZ);

  int botaos = digitalRead(34); //Le estado Push Button
  if (botaos)
    botao = !botao;

  digitalWrite(32, botao);  //Estado Buzzer
  digitalWrite(33, !botao); //Estado LED
  digitalWrite(25, botao);  //Estado OPTO1
  digitalWrite(26, !botao); //Estado OPTO2

  double T, P;                         //Temperatura e pressao BMP
  char result = bmp.startMeasurment(); //Medicao BMP
  if (result != 0)
  {
    delay(result);
    result = bmp.getTemperatureAndPressure(T, P);

    if (result != 0)
    {
      double A = bmp.altitude(P, P0);

      Serial.print("T = \t");
      Serial.print(T, 2);
      Serial.print(" degC\t");
      Serial.print("P = \t");
      Serial.print(P, 2);
      Serial.print(" mBar\t");
      Serial.print("A = \t");
      Serial.print(A, 2);
      Serial.println(" m");
    }
    else
    {
      Serial.println("Error.");
    }
  }
  else
  {
    Serial.println("Error.");
  }

  //Checa e mostra dados GPS
  while (Serial2.available() > 0)
    if (gps.encode(Serial2.read()))
      displayInfo();

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while (true)
      ;
  }

  // Verifica se temos o intervalo de tempo para enviar uma mensagem LoRa
  if (millis() - lastSendTime > interval)
  {
    String mensagem = " BORA SUPERNOVA "; // Definicao da mensagem
    sendMessage(mensagem);
    Serial.println("Enviando " + mensagem);
    lastSendTime = millis(); // Timestamp da ultima mensagem
  }
  // Procura por pacote e chama onReceive com o resultado
  onReceive(LoRa.parsePacket());

  delay(300); //Espera ate proximo loop (em milisegundos)
}