//Declarando funciones
void wifi_init(char*_ssid, char*_pass);
void wifi_ap_init(char*_ssid, char*_pass);

// Wifi en modo estacion
void wifi_init(char*_ssid, char*_pass){
  Serial.println("Conectando al AP" + String(_ssid));
  WiFi.mode(WIFI_STA); // Modo estacion ( por default)
  WiFi.begin(_ssid,_pass); // Inicia intento de conexion
  int wifi_cta = 0;
  while(WiFi.status()!= WL_CONNECTED && wifi_cta<15){
    Serial.print(".");
    delay(500);
    wifi_cta++;
    }

    Serial.println();
    if(wifi_cta<15)
    {
      #ifdef wifi_led
        digitalWrite(wifi_led, 1);
      #endif
      Serial.println("Conectado!");
      Serial.print("IP ->");
      Serial.println(WiFi.localIP());
      Serial.println();
      }
     else{
        Serial.println("Error -> No se logro la conexion");
        Serial.println("Reset en 3 segundos");
        delay(1000);
        Serial.println("Reset en 2 segundos");
        delay(1000);
        Serial.println("Reset en 1 segundos");
        delay(1000);
        Serial.println();
        ESP.restart();
        
        }
  }

// wifi en modo punto de acceso
void wifi_ap_init(char*_ssid, char*_pass){
  Serial.println("ESP32 COMO AP" + String(_ssid));
  WiFi.mode(WIFI_AP); // Modo AP
  WiFi.softAP(_ssid, _pass);
  Serial.println("Listo el ap");
  Serial.print("IP ->");
  Serial.println(WiFi.softAPIP());
  Serial.println();  
  
}