void Log(String s ="");
void Logn(String s);
void Logi(String s );
void Loge(String s );
void LogT(String s);
void log_marq(String s);

//IMPRIME LOG

void Log(String s ){Serial.print(s);}

//Imprime Nuevo renglon 

void Logn(String s ){Serial.println(s);}

//Imprime informacion

void Logi(String s ){Serial.println("info -->"+ s);}

//IMPRIME ERROR

void Loge(String s ){Serial.println("ERROR -->"+ s);}

//IMPRIME TITULOS

void LogT(String s ){s.toUpperCase();log_marq(s);}

//Marquesina 
void log_marq(String s){
  int tam=s.length();
  String msg= "";
  for(int ii=0; ii<tam+3; ii++)
  msg=msg + '*';

Serial.println();
Serial.println(msg);
Serial.println("* "+s+" *");
Serial.println(msg);
Serial.println();

  
}
