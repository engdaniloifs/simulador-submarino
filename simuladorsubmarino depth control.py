import numpy as np
import matplotlib.pyplot as plt
import time

# =============================================================================
# Classe Robot: Simula o comportamento do submarino e seu controle PID para a seringa.
# =============================================================================
class Robot:
    def __init__(self, x, z, submarinemass, submarinevolume, syringe_velocity, syringe_neutral):
        """
        Inicializa o robô (submarino).

        Parâmetros:
            x, z: Posições iniciais (m).
            submarinemass: Massa do submarino (kg).
            submarinevolume: Volume do submarino (m^3).
            syringe_velocity: Velocidade máxima de alteração da massa da seringa (g/s).
            syringe_neutral: Posição neutra da seringa (g).
        """
        self.x = x
        self.z = z
        self.syringemass = 0           # Massa atual da seringa (g)
        self.mass = submarinemass      # Massa do submarino (kg)
        self.volume = submarinevolume  # Volume do submarino (m^3)
        self.velocity_z = 0            # Velocidade no eixo z (m/s)
        self.acceleration = 0          # Aceleração no eixo z (m/s²)
        self.syringe_velocity = syringe_velocity
        self.syringe_neutral = syringe_neutral
        self.syringe_target = self.syringemass  # Posição alvo da seringa (g)
        self.setpointz = 0             # Profundidade desejada (m)
        self.servo_ang = 0

        # Variáveis para controle PID
        self.prev_error = 0
        self.error_integral = 0
        self.time_last_update = time.time()

    def update_motion(self, waterdensity, gravity, time_now, kp, ki, kd,
                      servo_passomax, servo_passomin, Ksa, depthmax):
        """
        Atualiza a dinâmica do robô, ajustando a massa da seringa via controle PID e
        atualizando a aceleração, velocidade e posição em profundidade.

        Parâmetros:
            waterdensity: Densidade da água (kg/m³).
            gravity: Aceleração da gravidade (m/s²).
            time_now: Tempo atual (s).
            kp, ki, kd: Ganhos do controlador PID.
            servo_passomax: Passo máximo permitido para o servo (g).
            servo_passomin: Passo mínimo para ativação do servo (g).
            Ksa: Fator de conversão entre massa e ângulo do servo.
            depthmax: Profundidade máxima (m).
        """
        # Calcula o intervalo de tempo (dt)
        dt = time_now - self.time_last_update
        self.time_last_update = time_now

        # Cálculo do erro em relação ao setpoint (profundidade desejada)
        error = (self.setpointz - self.z)

        # Atualiza o erro acumulado (integral)
        self.error_integral += error * dt

        # Calcula a derivada do erro
        error_derivative = (error - self.prev_error) / dt

        # Controlador PID para determinar a massa alvo da seringa
        self.syringe_target = (self.syringe_neutral +
                               (kp * error) +
                               (ki * self.error_integral) +
                               (kd * error_derivative))

        # Limita a variação da massa com base no passo máximo
        if abs(self.syringemass - self.syringe_target) > servo_passomax:
            if self.syringemass > self.syringe_target:
                self.syringe_target = self.syringemass - servo_passomax
            else:
                self.syringe_target = self.syringemass + servo_passomax

        # Se a variação for muito pequena, mantém a massa atual
        if abs(self.syringemass - self.syringe_target) < servo_passomin:
            self.syringe_target = self.syringemass

        # Conversão da massa da seringa para ângulo e volta para massa (para simulação)
        self.servo_ang = int(self.syringe_target * Ksa)
        self.syringe_target = self.servo_ang / Ksa

        # Garante que a massa alvo esteja dentro dos limites válidos
        self.syringe_target = max(0, min(self.syringe_target, 0.030))

        # Atualiza o erro para a próxima iteração
        self.prev_error = error

        # Atualiza a massa da seringa suavemente conforme a velocidade permitida
        if self.syringemass < self.syringe_target:
            self.syringemass = min(self.syringe_target, self.syringemass + self.syringe_velocity * dt)
        elif self.syringemass > self.syringe_target:
            self.syringemass = max(self.syringe_target, self.syringemass - self.syringe_velocity * dt)

    
        # Cálculo do arrasto (drag) na profundidade
        drag = 0.5 * (waterdensity * self.velocity_z * abs(self.velocity_z) *
                       0.82 * 0.11) / (self.mass + self.syringemass)

        # Atualiza a aceleração com base no empuxo e arrasto
        self.acceleration = -(gravity * ((waterdensity * self.volume) / (self.mass + self.syringemass) - 1) + drag)

        # Atualiza a velocidade e a posição em Z
        self.velocity_z += self.acceleration * dt
        self.z += self.velocity_z * dt

        # Limita a profundidade dentro dos limites do ambiente
        self.z = max(0, min(depthmax, self.z))
        if (self.z <= 0) or (self.z >= depthmax):
            self.velocity_z = 0


# =============================================================================
# Classe Environment: Gerencia a visualização do ambiente e informações do robô.
# =============================================================================
class Environment:
    def __init__(self, width, depth, aquario_img, submarine_img, robot):
        """
        Inicializa o ambiente de simulação.

        Parâmetros:
            width, depth: Dimensões do ambiente.
            aquario_img: Imagem do fundo (aquário).
            submarine_img: Imagem do submarino.
            robot: Instância do robô (submarino).
        """
        self.fig = plt.figure(figsize=(12, 6))

        # Subplots para o ambiente, gráficos e tabela de status
        self.ax1 = self.fig.add_axes([0.07, 0.55, 0.4, 0.4])  # Ambiente 2D do submarino
        self.ax4 = self.fig.add_axes([0.55, 0.55, 0.4, 0.4])  # Gráfico de profundidade ao longo do tempo
        self.ax2 = self.fig.add_axes([0.05, 0.3, 0.9, 0.1])  # Gráfico da seringa
        self.ax3 = self.fig.add_axes([0.37, 0.0, 0.25, 0.3])  # Tabela de dados

        # Configuração do ambiente 2D
        self.ax1.set_title('Tanque de testes')
        self.ax1.set_xlabel('Posição X (cm)')
        self.ax1.set_ylabel('Profundidade (cm)')
        self.ax1.set_ylim(100*depth, -10)
        self.ax1.set_xlim(0, 100*width)
        self.ax1.grid(True)
        self.setpoint_line, = self.ax1.plot([0, 120], [100*robot.setpointz, 100*robot.setpointz],
                                            'y--', lw=2, label='Setpoint')

        # Exibe as imagens do fundo e do submarino
        self.aquario_plot = self.ax1.imshow(aquario_img, extent=[0, 100*width, 100*depth, 0], aspect='auto', zorder=0)
        self.submarine_plot = self.ax1.imshow(submarine_img, extent=[0, 0, 0, 0], aspect='auto', zorder=1)

        # Configuração do gráfico da seringa
        self.ax2.set_title('Seringa')
        self.ax2.set_xlabel('Quantidade de água (ml)')
        self.ax2.set_xlim(0, 60)
        self.ax2.set_xticks(np.linspace(0, 60, 7))
        self.ax2.set_yticks([])
        self.syringe_line, = self.ax2.plot([], [], 'b-', lw=40)
        meio = robot.syringe_neutral*1000


        self.middle_line, = self.ax2.plot(
        [meio],
        [0],
        marker='|',
        markersize=40,
        color='red',
        linestyle='None',
       # label='Posição neutra'
        )
        self.ax2.legend(
        loc='upper left',
        bbox_to_anchor=(0.19, 1.7),  # desloca a legenda para fora do eixo
        #fontsize=10,
        title='Posição neutra',
        title_fontsize=10,
        #frameon=True,
        #fancybox=True,
        #framealpha=0.6
        )
        # Configuração da tabela de dados
        self.ax3.axis('tight')
        self.ax3.axis('off')
        data = [
            ['Profundidade (cm)', '{:.2f}'.format(100*robot.z)],
            ['Velocidade (cm/s)', '{:.2f}'.format(100*robot.velocity_z)],
            ['Aceleração (cm/s²)', '{:.2f}'.format(100*robot.acceleration)]
        ]
        self.table = self.ax3.table(cellText=data, loc='center', cellLoc='center')
        self.table.auto_set_font_size(False)
        self.table.set_fontsize(12)
        self.table.scale(1.3, 1.3)

        # Configuração do gráfico da profundidade ao longo do tempo
        self.ax4.set_title('Profundidade ao longo do Tempo')
        self.ax4.set_xlabel('Tempo (s)')
        self.ax4.set_ylabel('Profundidade (cm)')
        self.ax4.set_xlim(0, 60)
        self.ax4.set_ylim(100*depth, -10)
        self.ax4.grid(True)
        self.setpoint_line, = self.ax4.plot([0, 60], [100*robot.setpointz, 100*robot.setpointz],
                                            'y--', lw=2, label='Setpoint')
        self.depth_time_data = []      # Lista para tempos
        self.depth_position_data = []  # Lista para posições (profundidade)
        self.depth_line, = self.ax4.plot(self.depth_time_data,100* self.depth_position_data,
                                         color='b', lw=2, label="Posição do submarino")

        self.ax4b = self.ax4.twinx()
        self.ax4b.set_ylabel('Ângulo(°)')  # Ou outra unidade
        self.ax4b.set_ylim(0, 180)  # Ajuste conforme necessário

        self.angulo_data = []     # Lista da nova variável (ex: temperatura)

        self.angulo_line, = self.ax4b.plot(
            self.depth_time_data,
            self.angulo_data,
            color='r', lw=2, linestyle='-', label="Ângulo do servo motor"
        )

        lines_1, labels_1 = self.ax4.get_legend_handles_labels()
        lines_2, labels_2 = self.ax4b.get_legend_handles_labels()
        self.ax4b.legend(lines_1 + lines_2, labels_1 + labels_2, loc='upper right')

    def update_environment(self, robot, sim_time):
        """
        Atualiza as visualizações do ambiente com base na posição do robô.

        Parâmetros:
            robot: Instância do robô (submarino).
            sim_time: Tempo de simulação (s).
        """
        # Atualiza a posição da imagem do submarino
        img_extent = [100*(robot.x - 0.15), 100*(robot.x + 0.15), 100*(robot.z + 0.0675) , 100*(robot.z - 0.0675)]
        self.submarine_plot.set_extent(img_extent)

        # Atualiza o gráfico da seringa
        self.syringe_line.set_data([0, 1000*robot.syringemass-1.5], [0, 0])

        # Atualiza os dados na tabela
        self.table[(0, 1)].get_text().set_text('{:.2f}'.format(100*robot.z))
        self.table[(1, 1)].get_text().set_text('{:.2f}'.format(100*robot.velocity_z))
        self.table[(2, 1)].get_text().set_text('{:.2f}'.format(100*robot.acceleration))

        # Atualiza o gráfico de profundidade ao longo do tempo
        self.depth_time_data.append(sim_time)
        self.depth_position_data.append(100*robot.z)
        self.depth_line.set_data(self.depth_time_data, self.depth_position_data)

        self.angulo_data.append(robot.servo_ang)
        self.angulo_line.set_data(self.depth_time_data, self.angulo_data)

        self.ax4.relim()
        self.ax4.autoscale_view()
        plt.pause(0.0001)


# =============================================================================
# Função principal: Configura parâmetros, inicializa objetos e executa a simulação.
# =============================================================================
def main():
    # Configurações do ambiente
    env_width = 0.515
    env_depth = 0.315

    # Caminho para as imagens (ajuste conforme necessário)
    submarine_img_path = "D:\\Danilo\\projetos\\github\\simulador-submarino\\submarine.png"
    aquario_img_path = "D:\\Danilo\\projetos\\github\\simulador-submarino\\aquario.png"

    # Carrega as imagens
    submarine_img = plt.imread(submarine_img_path)
    aquario_img = plt.imread(aquario_img_path)

    # Posição inicial do submarino
    x = 0.2
    z = 0

    # Parâmetros do submarino
    submarine_volume = 0.0032  # m³
    submarine_mass = 3.185     # kg
    syringe_velocity = 0.015   # g/s
    syringe_neutral = 0.015    # g

    # Parâmetros do controle do servo
    servo_passomax = 0.0083    # g (equivalente a 50°)
    servo_passomin = 0.00083   # g (equivalente a 5°)
    Ksa = 6000                 # Fator de conversão (graus para gramas)

    # Ganhos do controlador PID 
    kp = 4
    ki = 0
    kd = 0

    # Parâmetros físicos
    waterdensity = 1000  # kg/m³
    gravity = 9.8        # m/s²

    # Aguarda o usuário iniciar a simulação
    while True:
        user_input = input("Digite 'start' para começar a simulação: ")
        if user_input.lower() == "start":
            print("Simulação iniciada!")
            break

    # Inicializa o robô (submarino) e define o setpoint de profundidade
    robot = Robot(x, z, submarine_mass, submarine_volume, syringe_velocity, syringe_neutral)
    robot.setpointz = 0.2  # Profundidade desejada (m)

    # Inicializa o ambiente gráfico
    environment = Environment(env_width, env_depth, aquario_img, submarine_img, robot)

    time_initial = time.time()

    # Loop principal de simulação
    try:
        while True:
            time_now = time.time()
            sim_time = time_now - time_initial

            # Atualiza a dinâmica do robô
            robot.update_motion(waterdensity, gravity, time_now,
                                kp, ki, kd, servo_passomax, servo_passomin, Ksa, env_depth)
            # Atualiza as visualizações do ambiente
            environment.update_environment(robot, sim_time)

            # Aguarda um curto intervalo para a próxima iteração
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("Simulação finalizada pelo usuário.")
        plt.show()

if __name__ == "__main__":
    main()
