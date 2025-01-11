Biblioteca Loramesh
Author: Renato F. Fernandes, Natan Ferreira Alvarenga, Hermes G. Fernandes Neri
Descricao:  
   - Codigo lora utilizando placas Heltec WIFI ESP32 lora V2 e V3. 
    (para compilar para uma placa V2 ou V3 basta alterar o arquivo platform.ini)
   - O codigo é baseado nas bibliotecas basicas da Heltec.
   https://github.com/HelTecAutomation/Heltec_ESP32/tree/master/examples
   e radiolib.h 

   -Inicio do codigo para rodar projeto loramesh (ainda em desenvolvimento)
   este codigo somente envia e recebe frames de diferentes tamanhos para 2 placas (ROUTER=1 envia e ROUTER=0 recebe).

V1_01
   - Foi implementado toda a parte de tratamento de dados no arquivo Loramesh
   - Agora nao eh necessario mais o define de ROUTER. Foi criado uma tabela com o DeviceID (serial number) de cada placa e enderecamento e tipo de devices fixos. 
     Neste caso, deve ser incluido cada placa nesta tabela.
   - Recepcao é feia atraves de interrupcao. 
   - Apesar da criacao da tarefa App_task, foi utilizado o tratamento da maquina de estado ainda no loop (estava tendo problema para enviar e receber os frames).
        

V1_00
   -Esta versao agora consegue enviar e receber os pacotes de forma mais estavel...
   Ele nao esta usando tarefas ainda, mas ele consegue enviar e receber de uma placa End device. 
   - Esta versao nao usa interrupcao na recepcao.

   
   
