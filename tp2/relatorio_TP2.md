# Teste de Performance - TP2 [OBRIGATÓRIO]
**Disciplina:** Fundamentos de Sistemas Robóticos com ROS 2
**Aluno:** Lucas Dias de Gondra

---

## Exercício 1: Publisher com geometry_msgs e inspeção de tópico

### Comparação entre mecanismos de comunicação

| Critério | Tópico | Serviço | Ação |
| :--- | :--- | :--- | :--- |
| **Sincronismo** | Assíncrono | Síncrono | Assíncrono (orientado a meta) |
| **Número de partes envolvidas** | N *Publishers* para M *Subscribers* (Muitos para Muitos) | 1 *Client* para 1 *Server* (1 para 1) | 1 *Client* para 1 *Server* (1 para 1) |
| **Feedback intermediário** | Contínuo / Não se aplica | Sem feedback intermediário (apenas resposta final) | Feedback contínuo durante a execução da meta |
| **Caso de uso típico** | Streaming contínuo de dados (câmera, LiDAR, `cmd_vel`) | Comandos pontuais e rápidos (ligar sensor, mudar estado) | Tarefas demoradas e complexas (navegação, mover braço) |

**Justificativa (Braço Robótico):** 
Para comandar um braço robótico a mover-se até uma posição com feedback de progresso, o mecanismo mais adequado é a **Ação (Action)**. Isso ocorre porque mover um braço é uma operação demorada que não é completada instantaneamente. A Ação permite que o programa solicite o movimento sem travar (*non-blocking*), e receba o feedback intermediário (como a porcentagem do percurso ou posição atual) ao longo do tempo, além de permitir cancelar a operação caso um obstáculo seja detectado ou o objetivo mude no meio do trajeto.

### Evidências e Validação (Insira seus prints aqui)
*(Para o relatório final, insira abaixo os prints da execução dos comandos após fazer o build do pacote)*

- Print de: `ros2 topic echo /cmd_vel`
- Print de: `ros2 topic hz /cmd_vel`
- Print de: `ros2 topic info /cmd_vel --verbose`

---

## Exercício 2: Subscriber com QoS e processamento de dados

### Perfis de QoS (Quality of Service)

**Diferenças:**
- **RELIABLE vs BEST_EFFORT (Reliability):**
  O perfil `RELIABLE` garante que as mensagens sejam entregues, reenviando-as em caso de perdas na rede, o que pode aumentar a latência da comunicação. Já o `BEST_EFFORT` não se preocupa em retransmitir as perdas, visando entregar os dados o mais rápido possível e minimizando a latência.
- **VOLATILE vs TRANSIENT_LOCAL (Durability):**
  O perfil `VOLATILE` significa que não há retenção de histórico para conexões futuras; um subscriber que acabou de se conectar só receberá as mensagens publicadas *após* a conexão. O perfil `TRANSIENT_LOCAL` instrui o publisher a guardar um histórico (buffer) das últimas mensagens publicadas e enviá-las para subscribers "atrasados" (late-joiners) no momento em que se conectam.

**Cenários Robóticos adequados para as combinações:**
1. **BEST_EFFORT + VOLATILE:** 
   Ideais para **streaming de dados de sensores de alta frequência**, como imagens de câmera ou LiDAR. Se um frame for perdido, não há vantagem em retransmiti-lo, pois o dado já estará obsoleto e o foco é no menor tempo de latência para a navegação do robô em tempo real.
2. **RELIABLE + VOLATILE:**
   Adequado para dados que precisam ser entregues com segurança, mas cujo histórico passado não é relevante para um nó que acabou de ligar. Exemplo: **Estado da bateria, telemetria importante do hardware ou alertas de diagnóstico contínuos**.
3. **RELIABLE + TRANSIENT_LOCAL:**
   Ideal para **Mapas estáticos (como o `/map` na stack do Nav2), Transforms (TFs) estáticas ou configurações iniciais**. Esses dados geralmente são publicados uma única vez e são essenciais; portanto, qualquer nó novo precisa recebê-los com garantia de entrega logo ao iniciar.
4. **BEST_EFFORT + TRANSIENT_LOCAL:**
   Normalmente pouco utilizado na prática, pois dados que exigem o histórico geralmente também requerem confiabilidade. Pode ser usado em ambientes de altíssimo ruído onde se deseja passar a última leitura de um sensor que varia lentamente, aceitando eventuais perdas na transmissão contínua para evitar congestionamento.

### Evidências e Validação (Insira seus prints aqui)
- Print do terminal do Subscriber rodando, exibindo a leitura com a velocidade linear calculada e o sentido de rotação (`horário` ou `anti-horário`).
- Print de: `ros2 node list` exibindo os nós `velocidade_pub` e `monitor_velocidade`.

---

## Exercício 3: Serviço Client/Server com std_srvs

### Fluxo completo de uma chamada de serviço ROS 2
1. **No Server:** O nó cria o serviço usando `create_service`, passando o tipo (ex: `SetBool`), o nome e a função de **callback**. Quando o Client envia a requisição, esse callback é acionado. O callback recebe dois objetos: `request` (com os dados de entrada) e `response`. Ele executa sua lógica, preenche os campos do `response` (ex: `success` e `message`) e o retorna.
2. **No Client:** O nó inicializa um client com `create_client`, utiliza `wait_for_service` para garantir que o server está rodando e monta a estrutura do `request`. Então, envia a chamada de forma assíncrona com **`call_async()`**.
3. **Aguardando Resposta:** O Client não fica travado; ele obtém um objeto `Future`. Com `rclpy.spin_until_future_complete()`, o executor gerencia a espera. Assim que a resposta chega, o Future é marcado como completo e seu resultado pode ser extraído.
4. **Evitando Deadlocks (MultiThreadedExecutor):** Quando usamos `call()` (síncrono) dentro de um callback (que por si só já está consumindo a *thread* principal do executor único padrão), o nó trava para sempre aguardando a resposta, pois não há outra thread livre para processá-la quando chegar. Por isso, usa-se `call_async()` e/ou o `MultiThreadedExecutor` (que possui múltiplas threads para gerenciar concorrência e processar retornos de serviço mesmo que outras partes estejam aguardando).

### Evidências e Validação (Insira seus prints aqui)
- Print de: `ros2 run comunicacao_ros2 modo_client -- true`
- Print de: `ros2 run comunicacao_ros2 modo_client -- false`
- Print de: `ros2 service list`

---

## Exercício 4: Isolamento por domínio DDS e inspeção do grafo

### Mecanismo DDS de Isolamento
O mecanismo responsável pelo isolamento é o DDS (*Data Distribution Service*). O ROS 2 utiliza o middleware DDS, no qual o `ROS_DOMAIN_ID` funciona como um identificador para criar domínios separados em uma mesma rede local. Fisicamente, o DDS mapeia esses IDs para portas UDP diferentes. Portanto, os dados de descoberta (discovery) e as mensagens publicadas em um domínio não afetam nem são visualizados por nós rodando em um ID diferente.

### Evidências e Validação (Insira seus prints aqui)

**1. Inspeção de Grafo:**
- Print de `ros2 node list`: *Mostra todos os nós atualmente operantes na rede para o domínio atual.*
- Print de `ros2 node info /velocidade_pub`: *Exibe os detalhes específicos do nó, como seus tópicos de publicação, assinaturas e serviços ofertados.*
- Print de `ros2 topic list -t`: *Exibe todos os tópicos ativos seguidos de seus respectivos tipos de mensagens (no caso, geometry_msgs/msg/Twist).*
- Print de `ros2 topic hz /cmd_vel`: *Permite verificar a taxa real de publicação de mensagens num tópico, garantindo que o timer configurado está cumprindo a frequência (2 Hz).*
- Print de `ros2 topic info /cmd_vel --verbose`: *Permite auditar detalhadamente quem está conectado ao tópico e as regras de QoS negociadas entre publishers e subscribers.*

**2. Domínios Diferentes:**
1. Altere o tópico no código do Publisher para `self.pub = self.create_publisher(Twist, '/robo1/cmd_vel', 10)`
2. Altere no código do Subscriber para assinar `/robo1/cmd_vel`.
3. *(Tire um Print)* Terminal 1 rodando Publisher: `ROS_DOMAIN_ID=42 ros2 run comunicacao_ros2 velocidade_pub`
4. *(Tire um Print)* Terminal 2 rodando Subscriber sem domínio definido (padrão 0): `ros2 run comunicacao_ros2 monitor_velocidade` 
  - *Note que as mensagens param de ser recebidas!*
5. *(Tire um Print)* Corrigindo e rodando no mesmo domínio: `ROS_DOMAIN_ID=42 ros2 run comunicacao_ros2 monitor_velocidade`
  - *Note que a comunicação é restabelecida!*
