// LIBRERÍAS PARA EL FUNCIONAMIENTO DEL SENSOR DE POSICIONAMIENTO
#include "I2Cdev.h"
#include "MPU6050.h"
// LIBRERÍA PARA EL FUNCIONAMIENTO DEL MÓDULO WIFI
#include "Wire.h"
// LA LIBRERÍA TINY GPS PERMITE OBTENER LONGITUD Y LATITUD DE UNA MANERA MÁS FÁCIL Y EVITAR UNA AMPLIA LÍNEA DE COMANDOS
#include <TinyGPS.h>
// CÓDIGO DE WIFI ----------------------------------------------------------------------------------------------------------------------
// SE INCLUYEN LA LIBRERÍA QUE PERMITE UNA CONFIGURACIÓN DEL EL MÓDULO WIFI P
#include <SoftwareSerial.h>
SoftwareSerial SerialESP8266(10,11); // RX, TX

//SE ASUME GRACIAS A LA LIBERÍA QUE LOS PINES SON 4 RX Y 3 TX PARA EL MÓDULO GPS NEO 6
TinyGPS gps;

// ESTE HABILITA EL RX Y TX DEL MÓDULO GPS
SoftwareSerial SerialGPS(4, 3); // RX, TX

String cadena="";
// VARIABLES PARA ENVIAR AL SERVIDOR
String ve="0";
String vs="1001";
float vl;  // ejem 19.239156;
float vlo;  // ejem-98.844177;

// FIN DE CÓDIGO WIFI ------------------------------------------------------------------------------------------------------------------
const int mpuAddress = 0x68;  // Puede ser 0x68 o 0x69
MPU6050 mpu(mpuAddress);
 
int ax, ay, az;
int gx, gy, gz;
 
long tiempo_prev;
float dt;
float ang_x, ang_y;
float ang_x_prev, ang_y_prev;
 
void updateFiltered()
{
   dt = (millis() - tiempo_prev) / 1000.0;
   tiempo_prev = millis();
 
   // CALCULAR LOS ÁNGULOS CON ACELERÓMETRO
   float accel_ang_x = atan(ay / sqrt(pow(ax, 2) + pow(az, 2)))*(180.0 / 3.14);
   float accel_ang_y = atan(-ax / sqrt(pow(ay, 2) + pow(az, 2)))*(180.0 / 3.14);
 
   // CALCULAR VÁNGULO DE ROTACIÓN CON GIROSCÓPIO Y FILTRO COMPLEMENTARIO
   ang_x = 0.98*(ang_x_prev + (gx / 131)*dt) + 0.02*accel_ang_x;
   ang_y = 0.98*(ang_y_prev + (gy / 131)*dt) + 0.02*accel_ang_y;
 
   ang_x_prev = ang_x;
   ang_y_prev = ang_y;
}

void coordenadas()
{
  //----------- CÓDIGO GPS - INICIO-----------------------------------------------
          bool nuevoDato = false;
          // LA VARIABLE UNSIGNED SOLO PERMITE VALORES POSITIVOS
          unsigned long caracteres;
          unsigned short sentencias, fallido;
        
          // POR UN SEGUNDO SE LEEN LOS DATOS DEL GPS Y SE REPORTAN ALGUNOS VALORES
          for (unsigned long start = millis(); millis() - start < 1000;)
          {
            while (SerialGPS.available())
            {
              char c = SerialGPS.read();
              if (gps.encode(c)) // LEE SI VIENE UN NUEVO VALOR
                nuevoDato = true;
            }
          }
        
          if (nuevoDato)
          {
            // LA LIBRERIA NECESITA DE UN FORMATO ASÍ COMO LA CANTIDAD DE NÚMEROS QUE PROPORCIONARÁ DESPUÉS DEL PUNTO 
            // DE ACUERDO A LA LIBRERÍA SOLO ES NECESARIO DECLARAR LAS VARIABLES FLOAT (LATITUD Y LONGITUD)
            // Y LA VARIABLE UNSIGNED (ESTA SOLO PERMITE VALORES POSITIVOS)
            // float flat, flon;
            unsigned long age;
    
            // CON ESTA LÍNEA LE ASIGNAMOS LOS VALORES DE LATITUD Y LONGITUD A LAS VARIABLEES flat y flon
            gps.f_get_position(&vl, &vlo, &age);
    
            // ESTAS LÍNEAS SIGUIENTES SOLAMENTE SON PARA IMPRIMIR LOS VALORES EN EL MONITOR SERIAL
            // LA LATITUD Y LA LONGITUD ESTAN ALMACENADAS EN LAS VARIABLES flat y flon respectivamente
            
            Serial.print("LATITUD=");
            Serial.print(vl == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : vl, 6);
            Serial.print(" LONGITUD=");
            Serial.print(vlo == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : vlo, 6);
  
          }         
   //----------- CÓDIGO GPS FINAL ------------------------------------------------- 
}

void setup()
{
   SerialESP8266.begin(9600);
   // Serial.begin(115200); // ESTE SE HABILITA PARA VER EL BUEN FUNCIONAMIENTO DEL MÓDULO GPS
   // ES IMPORTANTE SOLO HABILITAR UNA OPCION DEL Serial.begin()
   Serial.begin(9600); // ESTE SE PUEDE HABILITAR PARA VER EL BUEN FUNCIONAMIENTO WIFI DENTRO DEL MONITOR SERIAL

   // PARA INICIAR EL WIFI
   Wire.begin();
   mpu.initialize();
   Serial.println(mpu.testConnection() ? F("IMU iniciado correctamente") : F("Error al iniciar IMU"));
   
   // ESTE SOLO ES PARA ENCENDER UN LEDCITO QUE ENCIENDE CUANDO LA MOTOCICLETA SUFRE UNA CAIDA Y SE APAGA CUANDO SE
   // ENCUENTRA EN UN ESTADO NORMAL
   pinMode(2, OUTPUT);

   // SETUP WIFI ----------------------------------------------------------------------------------------------------
    SerialESP8266.setTimeout(2000);
    
    // VERIFICAMOS SE EL ESP8266 RESPONDE
    SerialESP8266.println("AT");
    if(SerialESP8266.find("OK"))
      Serial.println("Respuesta AT correcto");
    else
      Serial.println("Error en ESP8266");
  
    //-----CONFIGURACIÓN DE RED-------// LO PODEMOS COMENTAR SI EL MÓDULO ESP YA ESTA CONFIGURADO
  
      // ESP8266 EN MODO ESTACIÓN (NOS CONECTAREMOS A UNA RED EXISTENTE)
      SerialESP8266.println("AT+CWMODE=1"); // 3 MODO ESTACION
      if(SerialESP8266.find("OK"))
        Serial.println("ESP8266 en modo Estacion");
        
      // NOS CONECTAMOS A UNA RED WIFI 
      SerialESP8266.println("AT+CWJAP=\"INFINITUMnfyd\",\"999590a884\"");
      Serial.println("Conectandose a la red ...");
      SerialESP8266.setTimeout(10000); // AUMENTAR SI DEMORA LA CONEXIÓN
      if(SerialESP8266.find("OK"))
        Serial.println("WIFI conectado");
      else
        Serial.println("Error al conectarse en la red");
      SerialESP8266.setTimeout(2000);
      // DESHABILITAREMOS LAS CONEXIONES MÚLTIPLES
      SerialESP8266.println("AT+CIPMUX=0");
      if(SerialESP8266.find("OK"))
        Serial.println("Multiconexiones deshabilitadas");
    //------FIN DE CONFIGURACION-------------------
   // FIN DE SETUP WIFI ---------------------------------------------------------------------------------------------
}
 
void loop() 
{ 
   // OBTENEMOS LAS COORDENADAS
   // ENCENDEMOS GPS
   SerialGPS.begin(115200);
   coordenadas();
   
   // LEER LAS ACELERACIONES Y VELOCIDADES ANGULARES
   mpu.getAcceleration(&ax, &ay, &az);
   mpu.getRotation(&gx, &gy, &gz);

   // REALIZAMOS LOS CÁLCULOS
   updateFiltered();

   Serial.print(F("Rotacion en X:  "));
   Serial.print(ang_x);
   Serial.print(F("\t Rotacion en Y: "));
   Serial.println(ang_y);
 
   // delay(10);

   // CUANDO HAY UNA SOBREINCLINACIÓN EN LA PARTE IZQUIERDA---------------------------------------------------------------------------------
   if(ang_x<-50.000){
      digitalWrite(2, HIGH);
      ve="3";
   }
   else{
     // CUANDO HAY UNA SOBREINCLINACIÓN EN LA PARTE DERECHA---------------------------------------------------------------------------------
     if(ang_x>50.000){
        digitalWrite(2, HIGH);
        ve="3";
     }
     else{
      // CUANDO HAY UN ESTADO NORMAL DE LA MOTOCICLETA---------------------------------------------------------------------------------
        digitalWrite(2, LOW);
        ve="2";
     }
   }

   //---------ENVIAMOS LA VARIABLE AL SERVIDOR---------------------
   
   //NOS CONECTAMOS AL SERVIDOR:
   SerialESP8266.begin(9600);
          
   SerialESP8266.println("AT+CIPSTART=\"TCP\",\"192.168.1.65\",8080");
   if( SerialESP8266.find("OK"))
   {  
      Serial.println("ESP8266 conectado con el servidor...");
          
      //ARMAMOS EL ENCABEZADO DE LA PETICIÓN HTTP POR MÉTODO GET
      char buffer[10];
      String vlb = dtostrf(vl, 4, 6, buffer);  //4 is mininum width, 6 is precision
      String vlonb = dtostrf(vlo, 4, 6, buffer);
      String peticionHTTP= "GET /SARAM-API/public/api/saveDatos?ve="+String(ve)+"&vs="+String(vs)+"&lat="+String(vlb)+"&lon="+String(vlonb)+"\r\n";
      //peticionHTTP=peticionHTTP+String(valorEstado)+"&valorSARAM="+String(valorSARAM)+"HTTP/1.0\r\n";
      //peticionHTTP=peticionHTTP+"Host: 192.168.1.68\r\n";

      // ENVIAMOS EL TAMAÑO EN CARACTERES DE LA PETICIÓN HTTP:  
      SerialESP8266.print("AT+CIPSEND=");
      SerialESP8266.println(peticionHTTP.length());

      delay(1000);
      
      // ESPERAMOS A ">" PARA ENVIAR LA PETICION HTTP
      if(SerialESP8266.find(">")) // ">" INDICA QUE PODEMOS ENVIAR LA PETICION HTTP
      {
         Serial.println("Enviando HTTP . . .");
         SerialESP8266.println(peticionHTTP);
         if( SerialESP8266.find("SEND OK"))
         {  
            Serial.println("Peticion HTTP enviada:");
            Serial.println();
            Serial.println(peticionHTTP);
            Serial.println("Esperando respuesta...");
                    
            boolean fin_respuesta=false; 
            long tiempo_inicio=millis(); 
            cadena="";
                    
            while(fin_respuesta==false)
            {
               while(SerialESP8266.available()>0) 
               {
                  char c=SerialESP8266.read();
                  Serial.write(c);
                  cadena.concat(c);  // GUARDAMOS LA RESPUESTA EN EL STRING "cadena"
               }
               // FINALIZAMOS SI LA RESPUESTA ES MAYOR A 500 CARACTERES
               if(cadena.length()>10000) // PUEDEN AUMENTAR SI TIENEN SUFICIENTE ESPACIO EN LA MEMORIA
               {
                 Serial.println("La respuesta a excedido el tamaño maximo");
                          
                 SerialESP8266.println("AT+CIPCLOSE");
                 if( SerialESP8266.find("OK"))
                     Serial.println("Conexion finalizada");
                 fin_respuesta=true;
               }
               if((millis()-tiempo_inicio)>500) // FINALIZAMOS SI YA HAN TRANSCURRIDO MÁS DE 10 SEGUNDOS
               {
                 Serial.println("Tiempo de espera agotado");
                 SerialESP8266.println("AT+CIPCLOSE");
                 if( SerialESP8266.find("OK"))
                 Serial.println("Conexion finalizada");
                 fin_respuesta=true;
               }
               if(cadena.indexOf("CLOSED")>0) // SI RECIBIMOS UN CLOSED SIGNIFICA QUE HA FINALIZADO LA RESPUESTA
               {
                   Serial.println();
                   Serial.println("Cadena recibida correctamente, conexion finalizada");         
                   fin_respuesta=true;
               }
           }         
        }
        else
        {
           Serial.println("No se ha podido enviar HTTP.....");
        }            
      }
   }
   else
   {
      Serial.println("No se ha podido conectarse con el servidor");
   }

   //delay(1000);
   //-------------------------------------------------------------------------------
}
