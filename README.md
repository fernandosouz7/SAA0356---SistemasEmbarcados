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

A entrada de tensão (\(V_{\text{in}}(t)\)) para o PID seria a tensão aplicada ao motor (V(t)).

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

#### 2. Implementação do Controle PID:

   - Algoritmo PID: O código implementa o algoritmo de controle PID, compreendendo o cálculo do erro entre a referência e a leitura da velocidade simulada, juntamente com os termos integral e derivativo.

   - Ajuste dos Parâmetros PID: Os parâmetros do controlador PID (Kp, Ki, Kd) são ajustados conforme as características específicas do motor e da esteira, garantindo uma resposta eficaz e estável.

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

O gráfico gerado no Monitor Serial serve como uma ferramenta poderosa para entender o comportamento dinâmico do sistema de controle PID proposto, facilitando a otimização e aprimoramento do desempenho da esteira de academia.

