Biblioteca Loramesh
Author: Renato F. Fernandes e Natan Ferreira Alvarenga
Descricao:  
   - Codigo lora utilizando placas Heltec WIFI ESP32 lora V2 e V3. 
    (para compilar para uma placa V2 ou V3 basta alterar o arquivo platform.ini)
   - O codigo é baseado nas bibliotecas basicas da Heltec.
   https://github.com/HelTecAutomation/Heltec_ESP32/tree/master/examples
   
   -Inicio do codigo para rodar projeto loramesh (ainda em desenvolvimento)
   este codigo somente envia e recebe frames de diferentes tamanhos para 2 placas (ROUTER=1 envia e ROUTER=0 recebe).
   O limite de bytes que consegui enviar na V3 foi de 33 bytes...40 bytes nao envia...
   
- Esta versao implementa um timer event que peguei da internet no main.cpp. Porem o timer nao esta funcionando. 
Parece que o timer é baseado no RTC (64 kHz RC oscillator) porem eu nao vi que interrupcao ele gera (segundo o manual do SX1262 nao existe uma interrupcao para sleep mode)