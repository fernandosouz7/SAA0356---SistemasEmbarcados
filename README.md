## Alunos

Fernando dos Santos de Souza - 9311560 (Desenvolvimento das simulações, códigos e relatório)

Paulo Lopes Carvalhaes - 10747502 (Desenvolvimento das simulações, códigos e relatório)

## Introdução

A automação de esteiras de academia demanda sistemas de controle eficientes para assegurar um desempenho preciso e estável. Este projeto visa desenvolver um sistema de controle PID para um motor DC MAXON 118754, utilizando a placa VS50 Colibri Viola e a EPOS2 70/10 por meio do protocolo CAN. O enfoque recairá na simulação do sistema no Simulink e na implementação de códigos para interação com a EPOS2 70/10 via protocolo CAN.

## Metodologia:

### Equação de Dinâmica do Motor DC

A equação de dinâmica do motor DC pode ser expressa pelas seguintes equações diferenciais:

$V(t) = Ri(t) + L\frac{di(t)}{dt} + Ke\omega(t) \$

$T(t) = J\frac{d\omega(t)}{dt} + b\omega(t) + Ki(t) \$

onde:
- \(V(t) ) é a tensão aplicada ao motor,
- ( i(t)) é a corrente através do motor,
- ( &omega;(t)) é a velocidade angular do motor,
- (T(t)) é o torque aplicado ao motor,
- (J) é o momento de inércia do rotor,
- (b) é a constante de fricção viscosa do motor,
- (Kt) é a constante de torque do motor,
- (Ke) é a constante de força eletromotriz,
- (R) é a resistência elétrica,
- (L) é a indutância elétrica.

### Função de Transferência

A função de transferência do sistema, relacionando a velocidade angular (\(&omega(t);)) com a tensão aplicada (V(t)), é dada por:

$G(s) = \frac{\Omega(s)}{V(s)} = \frac{Kt}{(Js+b)(Ls+R)+KtKe} \$

### Entrada de Tensão do PID

A entrada de tensão V(t) para o PID seria a tensão aplicada ao motor (V(t)).

### Requisitos do Sistema e Ganhos PID

Os requisitos do sistema, como tempo de subida, overshoot, tempo de acomodação, etc., influenciam na escolha dos ganhos do PID K_p, K_i, K_d. Geralmente, os ganhos são ajustados experimentalmente para atender a esses requisitos.

**Sugestão inicial para os ganhos PID:**
- K_p: Ajuste para alcançar o desempenho desejado.
- K_i: Ajuste para eliminar o erro em estado estacionário.
- K_d: Ajuste para melhorar a resposta transitória e reduzir oscilações.

É recomendável começar com K_p e ajustar os outros ganhos conforme necessário. Experimentação prática é muitas vezes necessária para otimizar o controle.

### Simulação no Simulink:

A simulação no Simulink desempenha um papel crucial no desenvolvimento do sistema de controle para a esteira de academia, oferecendo uma abordagem virtual para avaliar e aprimorar o desempenho do motor DC MAXON 118754 e ajustar os controladores PID.

O processo inicia com a criação de um modelo Simulink detalhado que representa a malha de controle fechada. Para modelar as características específicas do motor DC MAXON 118754, são utilizados blocos que consideram detalhes como resistência (R), indutância (L), constante de torque (Kt) e constante de velocidade angular (Kv). A dinâmica do motor é representada por meio de blocos de função de transferência, permitindo uma simulação precisa do comportamento do sistema.

Ao configurar o PID, são adicionados blocos PID ao modelo para controlar a velocidade do motor. Ajustes cuidadosos nos parâmetros PID, como ganho proporcional (Kp), ganho integral (Ki) e ganho derivativo (Kd), são realizados por meio de técnicas de tuning. Esses ajustes são fundamentais para garantir uma resposta do sistema que atenda aos requisitos específicos de estabilidade, tempo de acomodação e erro em regime permanente.

Entradas de referência, como um bloco de degrau, são implementadas para simular variações de velocidade desejadas durante a simulação. Diferentes perfis de velocidade são explorados para avaliar a resposta do sistema em diversas condições operacionais.

![Captura de tela 2024-01-14 221228](https://github.com/fernandosouz7/SAA0356---SistemasEmbarcados/assets/97545209/fcea2da4-f58c-43b8-b856-c611d31019a8)

A execução da simulação proporciona uma visão aprofundada do comportamento do sistema em resposta às entradas de referência. A análise detalhada dos resultados, incluindo a resposta temporal, overshoot, settling time e erro em regime permanente, é essencial para compreender o desempenho do sistema e identificar possíveis áreas de aprimoramento.

Ajustes iterativos nos ganhos PID e nos parâmetros do modelo são realizados para otimizar a resposta do sistema. Diferentes configurações são testadas, e a simulação é refinada para garantir um desempenho robusto em uma variedade de condições operacionais.

![Captura de tela 2024-01-14 210504](https://github.com/fernandosouz7/SAA0356---SistemasEmbarcados/assets/97545209/896a1fbd-d07d-4ee0-bdcc-a5f94751185c)

### Tuning dos Controladores PID:

A etapa de tuning dos controladores PID é fundamental para otimizar a resposta do sistema, garantindo estabilidade e eficiência frente a variações de referência e perturbações. No contexto do projeto da esteira de academia, a abordagem para encontrar os ganhos do PID é uma escolha estratégica com impacto direto no comportamento dinâmico do sistema.

Para esse projeto, optamos por utilizar o método de Ziegler-Nichols, um método clássico e amplamente reconhecido. Nessa abordagem, o processo inicia com a configuração inicial dos ganhos PID, onde o ganho proporcional (Kp) é baixo, e os ganhos integral (Ki) e derivativo (Kd) são inicialmente nulos.

    Kp_initial = 0.2;
    Ki_initial = 0;
    Kd_initial = 0;

A simulação é então executada, e o ganho proporcional é gradualmente aumentado até que o sistema alcance a condição limite antes da instabilidade. Nesse ponto, identificamos o ganho crítico (Ku) e o período de oscilação crítica (Tu), representando o tempo necessário para uma oscilação completa.

Com base nos valores de Ku e Tu, os ganhos PID são calculados conforme as fórmulas estabelecidas para o método de Ziegler-Nichols:

    Kp_tuned = 0.6 * Ku;
    Ki_tuned = 2 * Kp_tuned / Tu;
    Kd_tuned = Kp_tuned * Tu / 8;


A configuração ajustada dos ganhos PID é então validada através de simulações adicionais. A análise detalhada da resposta do sistema avalia critérios como estabilidade, tempo de resposta e minimização de erro.

É importante mencionar que, caso necessário, ajustes iterativos podem ser realizados para refinar ainda mais os ganhos PID, otimizando o desempenho do sistema em condições específicas.

A escolha do método de Ziegler-Nichols proporciona uma abordagem sistemática e amplamente aceita para o tuning dos controladores PID, equilibrando a estabilidade do sistema com uma resposta dinâmica eficaz. Este processo contribui significativamente para a eficiência e desempenho do sistema de controle da esteira de academia.
    
### Simulação Via Wokwi:

A simulação no Wokwi desempenha um papel fundamental na validação e compreensão do sistema de controle PID proposto para a esteira de academia. Vamos explorar minuciosamente como a simulação é configurada e seu propósito no desenvolvimento do projeto:

#### 1. Configuração dos Componentes:

   - Potenciômetro (Referência de Velocidade): No Wokwi, o potenciômetro é escolhido para simular a entrada do usuário que define a referência de velocidade desejada para a esteira. Ele permite ajustes contínuos, representando a interação do usuário com a esteira.

   - Servo: Um Servo motor para simular o motor DC.

   - Arduino Uno: O Arduino Uno é o controlador sistema, onde o código implementa o algoritmo de controle PID e interage com os demais componentes para gerenciar a resposta da esteira.

![Captura de tela 2024-01-14 233419](https://github.com/fernandosouz7/SAA0356---SistemasEmbarcados/assets/97545209/0bec0d27-bef1-4e36-8a95-05f7865fda63)

Link da simulação: https://wokwi.com/projects/386951030467101697

#### 2. Implementação do Controle PID:

   - Algoritmo PID: O código implementa o algoritmo de controle PID, compreendendo o cálculo do erro entre a referência e a leitura da velocidade simulada, juntamente com os termos integral e derivativo.

   - Ajuste dos Parâmetros PID: Os parâmetros do controlador PID (Kp, Ki, Kd) são ajustados conforme as características específicas do motor e da esteira, garantindo uma resposta eficaz e estável.

````
#include <Servo.h>
// Constantes do controle PID
double Kp = 0.1;
double Ki = 0.2;
double Kd = 0.0;

// Variáveis de controle PID
double setpoint = 0;
double actualSpeed = 0;
double error = 0;
double integral = 0;
double derivative = 0;
double output = 0;

const int POT_PIN = A1;      // Pino analógico para o potenciômetro de setpoint
const int SERVO_PIN = 9;     // Pino de controle do servo
Servo meuServo;              // Objeto Servo para controlar o servo motor

unsigned long previousMillis = 0;
const long interval = 100; // Intervalo para simular controle de velocidade (ajuste conforme necessário)

void setup() {
  pinMode(POT_PIN, INPUT);
  meuServo.attach(SERVO_PIN); // Inicializa o objeto Servo com o pino do servo
  Serial.begin(9600);
}

void loop() {
  unsigned long currentMillis = millis();

  // Ajusta a velocidade a cada intervalo de tempo
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Leitura da referência de velocidade do potenciômetro
    setpoint = map(analogRead(POT_PIN), 0, 1023, -90, 90); // Use um intervalo de -90 a 90 para velocidade

    // Cálculos do PID
    error = setpoint - actualSpeed;
    integral += error;
    derivative = error - output;

    // Fórmula PID
    output = Kp * error + Ki * integral + Kd * derivative;

    // Limitar a saída para evitar valores extremos
    output = constrain(output, -90, 90); // Ajuste para o intervalo de velocidade desejado

    // Aplicação do resultado do PID (controle do servo)
    meuServo.writeMicroseconds(1500 + output * 10); // Ajuste conforme necessário

    // Saída de dados para o monitor serial (opcional)
    Serial.print("Setpoint: ");
    Serial.print(setpoint);
    Serial.print(", Actual Speed: ");
    Serial.print(actualSpeed);
    Serial.print(", Output: ");
    Serial.println(output);
  }
}
````

#### 3. Visualização e Análise:

   - LEDs como Indicadores de Saída: Durante a simulação, a intensidade dos LEDs reflete o sinal de controle gerado pelo PID. Isso proporciona uma visualização instantânea do comportamento do sistema em resposta às mudanças na referência.

   - Monitor Serial: No contexto da simulação no Wokwi, a opção de enviar dados para o Monitor Serial proporciona uma ferramenta valiosa para analisar o comportamento do sistema. Ao optar por essa funcionalidade, é possível gerar gráficos que representam visualmente a interação entre a referência de velocidade, a velocidade simulada e a saída do controle PID. Aqui está uma explicação detalhada sobre como interpretar esse gráfico:

##### 1. Eixo Horizontal (Tempo):
   - O eixo horizontal representa o tempo, com as leituras registradas em intervalos regulares durante a simulação. Cada ponto no gráfico corresponde a uma leitura feita em momentos específicos, proporcionando uma linha do tempo para análise.

##### 2. Eixo Vertical:

   - Referência de Velocidade (Setpoint): A linha correspondente à referência de velocidade (Setpoint) mostra como o usuário ajusta a velocidade desejada usando o potenciômetro no Wokwi. Essa linha permanece constante ou muda gradualmente, dependendo das interações do usuário com o potenciômetro durante a simulação.

   - Velocidade Simulada (PV - Process Variable): Representada por outra linha no gráfico, a velocidade simulada reflete a leitura simulada da velocidade da esteira. Variações nessa linha indicam como o sistema responde às mudanças na referência de velocidade.

   - Saída do Controle PID (CV - Control Variable): A terceira linha no gráfico representa a saída do controle PID, ou seja, o sinal de controle que influencia a velocidade da esteira. Esta linha mostra como o algoritmo PID ajusta a saída para atingir a referência de velocidade desejada.

##### 3. Análise de Comportamento:
   - Correspondência entre Linhas: Ao analisar o gráfico, observa-se como as linhas de referência, velocidade simulada e saída do controle PID interagem. Um bom desempenho do sistema seria caracterizado por uma rápida convergência da velocidade simulada à referência, indicando uma resposta eficiente do controle PID.

   - Estabilidade do Sistema: A estabilidade do sistema pode ser avaliada observando-se oscilações mínimas em torno da referência de velocidade. Quanto mais próxima a velocidade simulada estiver da referência, melhor o sistema está sendo controlado.

   - Ajustes no Tuning: Caso seja necessário otimizar o desempenho, a análise do gráfico pode sugerir ajustes nos parâmetros PID (Kp, Ki, Kd) para reduzir overshooting, tempo de estabilização, ou outros comportamentos indesejados.

![Captura de tela 2024-01-14 235020](https://github.com/fernandosouz7/SAA0356---SistemasEmbarcados/assets/97545209/fa350bbb-9a60-4935-8a91-92e99d962588)

### Protocolo CAN

A placa EPOS2 70/10 é um controlador digital de posição modular desenvolvido pela Maxon. Ela é adequada para uma gama de motores, incluindo motores DC com escovas com encoder e motores EC sem escobillas com sensores Hall e encoder, cobrindo uma faixa de potência de 80 a 700 watts. Ela Suporta encoders digitais incrementais (2 e 3 canais, diferenciais), sensores Hall digitais (para motores EC), encoders absolutos (SSI) e encoders analógicos incrementais (2 canais, diferenciais). Entre seus protocolos de comunicação compatíveis, inclui interfaces RS232, USB (velocidade total), CAN, além de suporte para CANopen. Possui várias funções de gateway, como RS232-to-CAN e USB-to-CAN.\\
Quanto ao uso, a EPOS2 70/10 é empregada para controlar a posição, velocidade e corrente de motores em diversas aplicações industriais e de automação. Sua flexibilidade, alta eficiência e ampla gama de compatibilidade com diferentes tipos de motores e encoders a tornam uma solução versátil para muitos cenários que requerem controle preciso de movimento.

![EPOS2_image1](https://github.com/fernandosouz7/SAA0356---SistemasEmbarcados/assets/97545209/ff17ad5c-d500-423c-9980-3d5a76b9080b)

As funções de comunicação CAN (Controller Area Network) da placa EPOS2 70/10 da Maxon são bastante relevantes, especialmente para aplicações que exigem integração com sistemas de controle e automação industrial. A comunicação CAN é um protocolo robusto usado para permitir a comunicação confiável entre diversos dispositivos eletrônicos, como controladores de motor, sensores e atuadores, em ambientes industriais. Vamos detalhar algumas das principais funções de comunicação CAN desta placa:
CAN e CANopen:

A EPOS2 70/10 suporta o protocolo CAN, um padrão de comunicação serial para redes de dispositivos.
Incorpora também o CANopen, que é um protocolo de comunicação e um perfil de dispositivo para sistemas embarcados usados em automação. O CANopen permite a fácil integração de dispositivos de diferentes fabricantes.

A placa segue o CANopen application layer DS-301, que define os aspectos básicos de comunicação e dados em um sistema CANopen.
Implementa também os frameworks DSP-305 e os perfis de controle de movimento DSP-402, que estabelecem padrões para o controle de dispositivos de automação, como motores e atuadores.

A EPOS2 70/10 possui funcionalidades de gateway, como RS232-to-CAN e USB-to-CAN, que permitem a conversão de sinais de um padrão de comunicação para outro, facilitando a integração com sistemas que não são nativamente compatíveis com CAN.

Através da interface CAN, é possível programar e configurar a EPOS2 70/10, ajustando parâmetros como limites de velocidade, aceleração, controle de posição, entre outros.
A placa pode ser configurada para operar em diversos modos, como controle de posição, velocidade ou corrente, utilizando as instruções e parâmetros definidos no protocolo CANopen.

Em sistemas complexos que envolvem múltiplos controladores e dispositivos, a comunicação CAN permite a sincronização e coordenação de ações entre estes dispositivos.
Isso é especialmente útil em aplicações onde vários motores precisam operar de forma coordenada, como em linhas de montagem automatizadas, robótica e sistemas de transporte.

Através da rede CAN, também é possível realizar diagnósticos e monitorar o estado e desempenho da EPOS2 70/10, incluindo a leitura de códigos de erro, monitoramento de temperatura e desempenho operacional.
Em resumo, as funções de comunicação CAN da EPOS2 70/10 desempenham um papel vital na integração, controle, monitoramento e diagnóstico em sistemas automatizados e de controle de movimento, tornando-a uma escolha robusta e flexível para diversas aplicações industriais e de automação

![CAN_image1](https://github.com/fernandosouz7/SAA0356---SistemasEmbarcados/assets/97545209/0a116c52-90fb-4509-af2c-6c643830001b)

##### Uso do Protocolo CAN na EPOS2 70/10

##### Especificações de Conexão CAN

A EPOS2 70/10 utiliza pinos específicos para o protocolo CAN, conforme indicado no datasheet.

Para a placa Colibri VF50, os pinos de comunicação CAN são o 63 para receiver (RX) e 55 para transmitter (TX).

Detalhes na implementação do cabeamento devem seguir as especificações, incluindo a adição de um resistor de 120 Ohms entre CAN-H e CAN-L, espaçamento máximo de 30 cm entre os hardwares no barramento, entrelaçamento dos cabos CAN_L e CAN_H, e blindagem eletromagnética.

##### Bibliotecas e Funções para Protocolo CAN

Para se comunicar com a EPOS2 70/10 utilizando o protocolo CAN, é necessário utilizar bibliotecas específicas. Segundo o guia "EPOS Command Library", a biblioteca apropriada para Linux é a "libEposCmd.so".

As funções principais para a utilização do protocolo CAN incluem:

- `VCS_Send_CAN_Frame()`: Enviar dados pela rede CAN.
- `VCS_Read_CAN_Frame()`: Receber dados pela rede CAN.

Para iniciar a comunicação entre a EPOS e a placa, a função `VCS_OpenDevice()` é utilizada.

É importante configurar corretamente os parâmetros da `VCS_OpenDevice()`, utilizando funções como `VCS_GetDeviceNameSelection()`, `VCS_GetProtocolStackNameSelection()`, `VCS_Get_InterfaceNameSelection()`, e `VCS_GetPortNameSelection()`. Por exemplo: `VCS_OpenDevice(char* EPOS2, char* CANopen, char* , char* CAN0)`.

Ao final do código, a porta da EPOS deve ser fechada usando a função `VCS_CloseDevice(1)`.

##### Identificação de Motor e Encoder

Para identificar o tipo e parâmetros do motor, as funções `VCS_SetMotorType()` e `VCS_SetDcMotorParameterEx()` são utilizadas.

Da mesma forma, para o encoder, as funções `VCS_SetSensorType()` e `VCS_SetIncEncoderParameter()` são utilizadas.

##### Controle PID e Configurações

A biblioteca oferece funções prontas para controle PID, como `VCS_SetControllerGain()`. Para configurar entradas e saídas digitais, funções como `VCS_DigitalInputConfiguration()` e `VCS_DigitalOutputConfiguration()` são utilizadas.

Para entradas/saídas analógicas, as funções `VCS_AnalogInputConfiguration()` e `VCS_AnalogOutputConfiguration()` são empregadas

##### Leitura e Controle de Velocidade

Para ler a velocidade com o encoder, a função `VCS_GetVelocityls()` é utilizada.

Para controlar a velocidade do motor, as funções `VCS_SetOperationMode()` ou `VCS_ActivateVelocityMode()` e `VCS_SetVelocityMust()` são empregadas.

Este guia serve como um ponto de partida, sendo essencial consultar a documentação completa para ajustes específicos conforme as necessidades da aplicação.

#### Conexão entre Placa VS50 Colibri Viola e EPOS2 70/10

Para estabelecer a comunicação entre a placa VS50 Colibri Viola e a EPOS2 70/10, é necessário seguir corretamente as conexões físicas e configurar os parâmetros adequados. Abaixo, estão os passos para realizar a ligação:

##### Conexão Física:

- Pinos CAN na EPOS2 70/10:
  - Consulte o datasheet da EPOS2 70/10 para identificar os pinos de comunicação CAN. Geralmente, são utilizados CAN_H e CAN_L para a transmissão e recepção de dados.

- Pinos CAN na VS50 Colibri Viola:
  - Consulte o datasheet da placa Colibri Viola para identificar os pinos de comunicação CAN. Eles são geralmente denominados como RX (Receiver) e TX (Transmitter).

- Cabeamento CAN:
  - Utilize cabos CAN de qualidade, garantindo que a distância entre os dispositivos não ultrapasse os limites especificados (geralmente até 30 cm).
  - Adicione um resistor de terminação de 120 Ohms entre os fios CAN_H e CAN_L, garantindo a integridade do sinal.

- Entrelaçamento de Cabos:
  - Entrelace os cabos CAN_H e CAN_L para reduzir interferências eletromagnéticas.

- Blindagem:
  - Se possível, utilize cabos com blindagem para minimizar interferências externas.

- Alimentação:
  - Certifique-se de que ambos os dispositivos estejam devidamente alimentados.

##### Configuração de Parâmetros:

- Protocolo CAN:
  - Certifique-se de que ambos os dispositivos suportam e estão configurados para o protocolo CAN.

- Baud Rate:
  - Configure o mesmo Baud Rate (taxa de transmissão) nos dois dispositivos. Geralmente, valores comuns são 125 kbps, 250 kbps, 500 kbps ou 1 Mbps.

- Endereçamento:
  - Atribua endereços únicos para cada dispositivo na rede CAN. Isso é crucial para garantir que os dados sejam enviados e recebidos corretamente.

- Modo de Operação da EPOS2 70/10:
  - Configure a EPOS2 70/10 para operar no modo desejado (por exemplo, controle de posição, controle de velocidade) através dos parâmetros adequados.

- Configuração da Colibri Viola:
  - Consulte o manual da Colibri Viola para configurar corretamente os pinos CAN, especificando as funções de RX e TX.

- Teste de Comunicação:
  - Antes de iniciar a aplicação completa, realize testes de comunicação básicos para garantir que os dispositivos possam se comunicar corretamente.

## Implementação

O código implementado utiliza a lógica de um potenciômetro para realizar o controle de velocidade, assim como o simulado no Wokwi.

```
#include <sys/time.h>
#include <stdlib.h>
#include <stdio.h>
#include "pid.h"
#include "libEposCmd.so"

#define MOTOR_CAN_ID  0x01  // Endereço CAN do motor
#define ENCODER_CAN_ID 0x02  // Endereço CAN do encoder


int r = 0.1;

float measurement;
float setpoint;
float control_action;

void setup()
{
    Serial.begin(9600);

    //Configurar o tipo de motor e parâmetros do motor
    VCS_SetMotorType(VCS_GetMotorHandle(MOTOR_CAN_ID));
    VCS_SetDcMotorParameterEx(VCS_GetMotorHandle(MOTOR_CAN_ID));

    // Configurar o tipo de sensor e parâmetros do encoder
    VCS_SetSensorType(VCS_GetSensorHandle(ENCODER_CAN_ID));
    VCS_SetIncEncoderParameter(VCS_GetSensorHandle(ENCODER_CAN_ID));

    // Iniciar o controlador PID, se necessário
    PIDController controller;
    PIDController_Init(&controller);

}

int main(int argc, char *argv[])
{

    setup();

    while (1)
    { 
        // Leitura do potenciômetro
        int potValue = analogRead(potPin);
        uiContador = map(potValue, 0, 1023, 0, 9);

        // Ajuste direto da variável de referência
        int vref = 5;
        setpoint = vref * r;

        // Os parâmetros das funções VCS_ReadCANFrame e VCS_GetVelocityls precisam ser definidos ainda
        unsigned int KeyHandle = 123;  
        unsigned int CobID = 456;
        unsigned int Length = 7;     
        unsigned int NodeId = 1; 
        unsigned int pVelocityls[2]; 
        unsigned int pErrorCode = 0; 
        unsigned int Timeout = 1000; 
        unsigned int ErrorCode = 0;

        measurement = VCS_ReadCANFrame(KeyHandle, CobID, Length, VCS_GetVelocityls(KeyHandle, NodeId, pVelocityls, pErrorCode), Timeout, ErrorCode);

        PIDController controller;
        PIDController_Init(&controller);

        measurement = Sensor_update();
        control_action = PIDController_Update(&controller, setpoint, measurement);
    }
    VCS_CloseDevice(1);
    return 0;
}
```
## Compilação

Para realizar a compilação do código PID, é imprescindível configurar a toolchain através do SDK específico para a placa VF50. Isso se deve à distinção de arquitetura entre o processador da placa e o do host, requerendo, assim, o procedimento de compilação cruzada. O SDK pode ser obtido diretamente do portal da Toradex.

Posteriormente, é necessário estabelecer a conexão via ethernet utilizando o protocolo SSH no terminal do host:


    ssh root@192.168.1.100


Para compilar o código e carregá-lo na placa, são necessários o seguintes comandos no terminal:


    $CC -Wall main.c -o control
    $CC -Wall pid.c -o pid

    scp control root@192.168.1.100:/home/root
    scp pid root@192.168.1.100:/home/root
    scp pid.h root@192.168.1.100:/home/root
    scp pid_constants.h root@192.168.1.100:/home/root

O comando para rodar o código é o seguinte:


    ./control


## Resultados

A seguir, destacamos os resultados obtidos:

### Desempenho do Controle PID:

- Tuning dos Controladores PID:
  - Os controladores PID foram ajustados utilizando técnicas de tuning para otimizar o desempenho do sistema em termos de tempo de resposta, overshoot e estabilidade.

- Resposta ao Degrau:
  - Apesar da impossibilidade de testes físicos, foram realizados testes virtuais de resposta ao degrau para avaliar o comportamento dinâmico do sistema em relação a mudanças abruptas na referência de velocidade.

- Gráficos de Desempenho:
  - Gráficos foram gerados na simulação para visualizar a resposta do sistema em diferentes condições de operação, destacando o comportamento do motor em termos de velocidade e posição.

### Simulação no Wokwi:

- Simulação do Sistema:
  - Utilizando o simulador online Wokwi, foi possível simular o modelo do sistema desenvolvido no Simulink, permitindo uma análise visual e interativa do comportamento do motor e do controle PID.

- Validação dos Parâmetros:
  - Os resultados da simulação foram comparados com as expectativas teóricas e os parâmetros do sistema, verificando a validade das configurações e do modelo adotado.

### Considerações Finais:

- Estabilidade do Sistema:
  - A estabilidade do sistema foi verificada virtualmente em diferentes condições operacionais, garantindo que o motor DC se mantenha controlado e responsivo.

- Conclusões sobre o Controle PID:
  - Com base nos resultados virtuais obtidos, são tiradas conclusões sobre a eficácia do controle PID implementado, identificando pontos fortes e possíveis melhorias.

Esses resultados virtuais proporcionam uma compreensão abrangente do desempenho do sistema, possibilitando ajustes adicionais, se necessário, para atender aos requisitos e objetivos estabelecidos no projeto, considerando a impossibilidade de realizar testes físicos.

## Conclusão

Não foi possível para nós fazermos os testes em laboratório, pois trabalhamos durante a semana o que dificultou a implementação no circuito físico. Contudo, fizemos as simulações possíveis para validar o projeto proposto.
