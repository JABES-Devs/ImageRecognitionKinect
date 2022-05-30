# ImageRecognitionKinect
Image Recognition using Kinect and Cascade Classifier

PT-BR

Para o funcionamento deste projeto, e necessario que exista um arquivo
.properties com os diretorios de recursos utilizados pelo sistema. Com
estes diretorios, o sistema ira buscar e gravar do sistema e do Kinect.

Ainda, alem de diretorios, no arquivo .properties estao parametros de 
emissao de topicos ROS, tal como o intervalo de envio de mensagens para
determinado simbolo.

Por padrao, este arquivo esta localizado em
"./resources/image-recognition_config.properties".
Desta forma, este arquivo esta imediatamente dentro da pasta raiz do projeto.
Caso deseje alterar o diretorio deste arquivo properties, sera necessario
ajustar sua chamada no programa AppInitEntry.py

Todos os outros recursos especificados no arquivo .properties precisam
estar localizados em algum local DENTRO da pasta do projeto.
Exemplo: o arquivo haarcascade .XML precisa estar incluido
em alguma pasta DO PROJETO, nao em alguma pasta externa do sistem
ou em algum outro usuario.
