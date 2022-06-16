# ImageRecognitionKinect
Image Recognition using Kinect and Haar Cascade Classifier

PT-BR

Primeiramente, conecte o Kinect ao sistema, e execute o comando em terminal:
freenect-micview

Para iniciar o sistema, basta executar o comando
rosrun kinect_detection_pkg src/kinect_detection_pkg/python/entry/AppInitEntry.py


DEMAIS INFORMAÇÕES

ARQUIVO .PROPERTIES

Para o funcionamento deste projeto, é necessário que exista um arquivo
.properties com os diretorios dos recursos utilizados pelo sistema,
além de outras informações sobre propriedades de hardware e do sistema.
A partir dele, o sistema poderá operar corretamente para captar e processar
imagens da câmera - neste caso, o Kinect V1 1473.

Por padrão, este arquivo esta localizado em
"./resources/image-recognition_config.properties".
Caso deseje alterar o diretório deste arquivo .properties, sera necessário
ajustar sua chamada no programa AppInitEntry.py.

Neste arquivo, estão inclusas informações sobre localização do classificador
Haar Cascade, propriedades das câmeras, e parâmetros de processamento e emissão
de mensagens em tópicos ROS.

Os diretórios de recursos do arquivo .properties partem da pasta ROOT do projeto,
ou seja, esta pasta: a pasta onde está localizado o arquivo README.


ROS PACKAGE

ATENÇÃO: o pacote ROS deste projeto está estruturado de forma que seus arquivos
estão visíveis para outros pacotes - principalmente, para si próprio.
Isto foi feito para que os procedimentos do sistema, separados em diferentes
arquivos .py, possam ser funcionar em conjunto.

Esta solução requer uma estrutura específica de pastas que pode confundir:
Na pasta SRC do workspace está o "ROS Package", e na pasta SRC do package, está
a "Pasta do Projeto".

ROS_Workspace/ src / kinect_detection_pkg / src / kinect_detection_pkg /

Os nomes das pastas do pacote ROS e do projeto precisam ser iguais.
Caso deseje alterar o nome do pacote ROS, será necessário ajustar a pasta interna
e os arquivos de configuração como o CMakeList.txt, Package.xml e Setup.py, além
de ajustar instruções de importação nos arquivos internos do sistema.


LIBFREENECT

Este projeto faz uso de drivers open-source e de seu wrapper em python3,
o libfreenect, para o Kinect v1.

Sua instalação, e de suas dependencias, deve ser feita no sistema Linux.
Após a instalação efetuada segundo instruções disponibilizadas no documento
do projeto - e também no github open-source - será necessário gerar arquivos
de firmware de áudio e movê-los para um diretório específico.
(Instruções completas disponibilizadas no documento do projeto)

Esta etapa é importante pois sem o firmware de áudio, o kinect não consegue
inicializar corretamente e não irá funcionar.
Ainda, além de "gerar e posicionar o firmware", será preciso carregá-lo
toda vez que o kinect for conectado ao computador, com o comando em terminal:
freenect-micview.
Este comando irá executar um exemplo pronto, instalado com o driver, que
testa os microfones do aparelho - mas que, crucialmente, carrega o firmware.

